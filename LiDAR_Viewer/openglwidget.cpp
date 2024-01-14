#include "openglwidget.h"
#include <QDebug>
OpenGLWidget::OpenGLWidget(QWidget *parent): QOpenGLWidget(parent)
{
    is_connected_ = false;
    // 初始化网络服务
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    // 创建定时器请求点云信息
    timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(updatePointCloudData())); // 设置定时器响应函数
}

OpenGLWidget::~OpenGLWidget()
{
    closesocket(sock);
    WSACleanup();
}

bool OpenGLWidget::connectToServer(QString address,quint16 port)
{
    if(is_connected_)
        return true;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr;
    memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.S_un.S_addr = inet_addr(address.toStdString().c_str());

    if(::connect(sock,(sockaddr*)&addr,sizeof(sockaddr_in))==SOCKET_ERROR)
    {
        closesocket(sock);
        return false;
    }
    is_connected_ = true;
    ctl_queue.push_back(ControlMsg::GETCONFIG);
    timer->start(66);
    return true;
}

void OpenGLWidget::closeConnect()
{
    closesocket(sock);
    timer->stop();
    is_connected_ = false;
    ctl_queue.clear();
    QMessageBox::information(this,"info","Disconnect.");
}

void OpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);

    type.static_update = type.safe_update = type.warning_update = true;

    for(int i=0;i<3;i++) // 初始化点云数据
        socket_datas[i].len = 0;
    box_datas.len = 0;

    // 初始化相机
    fov = 45.0;
    cam_position = QVector3D(0.0f,0.0f,20.0f);
    cam_front = QVector3D(0.0f,0.0f,-1.0f);
    cam_up = QVector3D(0.0f,1.0f,0.0f);

    QString vertex_files[3] = {":/shaders/static_pointcloud.vert",
                               ":/shaders/safe_pointcloud.vert",
                               ":/shaders/warning_pointcloud.vert"};
    QString fragment_files[3] = {":/shaders/static_pointcloud.frag",
                                ":/shaders/safe_pointcloud.frag",
                                ":/shaders/warning_pointcloud.frag"};

    glGenVertexArrays(3,VAOs);
    glGenBuffers(3,VBOs);
    for(int i=0;i<3;i++)
    {
        // 2.绑定VAO，开始记录属性相关
        glBindVertexArray(VAOs[i]);
        // 3.绑定VBO(一定是先绑定VAO再绑定VBO)
        glBindBuffer(GL_ARRAY_BUFFER,VBOs[i]);
        // 4.把数据放进VBO
        glBufferData(GL_ARRAY_BUFFER,socket_datas[i].len*sizeof(Point),socket_datas[i].points,usages[i]);
        // 5.解析数据
        glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),(void*)0);
        // 6.开启location = 0的属性解析
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER,0);

        // 设置渲染器
        shaders[i].addShaderFromSourceFile(QOpenGLShader::Vertex,vertex_files[i]);
        shaders[i].addShaderFromSourceFile(QOpenGLShader::Fragment,fragment_files[i]);
        shaders[i].link();
    }

    // bounding box渲染
    glGenVertexArrays(1,&box_VAO);
    glGenBuffers(1,&box_VBO);
    glBindVertexArray(box_VAO);
    glBindBuffer(GL_ARRAY_BUFFER,box_VBO);
    glBufferData(GL_ARRAY_BUFFER,box_datas.len*sizeof(Point),box_datas.points,GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER,0);
    box_shader.addShaderFromSourceFile(QOpenGLShader::Vertex,":/shaders/bounding_box.vert");
    box_shader.addShaderFromSourceFile(QOpenGLShader::Fragment,":/shaders/bounding_box.frag");
    box_shader.link();

    // 坐标背景渲染
    float coordinate_points[(360*3+8)*2];
    unsigned int index[3],line_index = 360*3*2;
    for(int i=0;i<3;i++)
        index[i] = i*360*2;
    for(int i=0;i<360;i++)
    {
        double rad = i*M_PI/180;
        for(int j=0;j<3;j++)
        {
            coordinate_points[index[j]++] = (j+1)*4*cos(rad); // x
            coordinate_points[index[j]++] = (j+1)*4*sin(rad); // y
        }
    }
    for(int i=0;i<180;i+=45)
    {
        double rad = i*M_PI/180;
        coordinate_points[line_index++] = 13*cos(rad); // x
        coordinate_points[line_index++] = 13*sin(rad); // y
        coordinate_points[line_index++] = -13*cos(rad); // -x
        coordinate_points[line_index++] = -13*sin(rad); // -y
    }
    glGenVertexArrays(1,&coordinate_VAO);
    glGenBuffers(1,&coordinate_VBO);
    glBindVertexArray(coordinate_VAO);
    glBindBuffer(GL_ARRAY_BUFFER,coordinate_VBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(coordinate_points),coordinate_points,GL_STATIC_DRAW);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER,0);

    coordinate_shader.addShaderFromSourceFile(QOpenGLShader::Vertex,":/shaders/coordinate.vert");
    coordinate_shader.addShaderFromSourceFile(QOpenGLShader::Fragment,":/shaders/coordinate.frag");
    coordinate_shader.link();
}

void OpenGLWidget::resizeGL(int w,int h)
{
    width = w;
    height = h;
    glViewport(0,0,width,height);
}

void OpenGLWidget::paintGL()
{
    // 设置窗口颜色
    glClearColor(0.1f,0.1f,0.1f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 view,projection;
    view.lookAt(cam_position,cam_position+cam_front,cam_up);
    projection.perspective(fov,width/height,0.1f,100.0f);

    // 绘制bounding_box
    if(box_datas.len>0 && box_check)
    {
        qDebug()<<"to shade bounding box";
        box_shader.bind();
        box_shader.setUniformValue("view",view);
        box_shader.setUniformValue("projection",projection);
        glBindVertexArray(box_VAO);
        int count = box_datas.len/4;
        qDebug()<<"count:"<<count;
        for(int i=0;i<count;i++)
            glDrawArrays(GL_LINE_LOOP,i*4,4); // 绘制矩形框
        qDebug()<<"shade bounding box";
    }


    for(int i=2;i>=0;i--)
        if(socket_datas[i].len>0 && checks[i]) // 无数据或不显示时不进行渲染
        {
            shaders[i].bind();
            shaders[i].setUniformValue("view",view);
            shaders[i].setUniformValue("projection",projection);
            glBindVertexArray(VAOs[i]);
            glDrawArrays(GL_POINTS,0,socket_datas[i].len);
        }

    coordinate_shader.bind();
    coordinate_shader.setUniformValue("view",view);
    coordinate_shader.setUniformValue("projection",projection);
    glBindVertexArray(coordinate_VAO);
    for(int i=0;i<3;i++) // 画圆
        glDrawArrays(GL_LINE_LOOP,i*360,360);
    glDrawArrays(GL_LINES,3*360,8);
}

void OpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    last_pos = event->pos();
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->pos().x() - last_pos.x();
    int dy = event->pos().y() - last_pos.y();
    if(event->buttons() & Qt::LeftButton)
    {
        float new_x = cam_position.x() - dx*0.05f;
        if(new_x>10)
            new_x = 10;
        else if(new_x<-10)
            new_x = -10;
        float new_y = cam_position.y() + dy*0.05f;
        if(new_y>10)
            new_y = 10;
        else if(new_y<-10)
            new_y = -10;

        cam_position.setX(new_x);
        cam_position.setY(new_y);
    }
    if(event->buttons() & Qt::RightButton)
    {
        float new_z = cam_position.z() + dy*0.05f;
        if(new_z>30)
            new_z = 30;
        else if(new_z<5)
            new_z = 5;

        cam_position.setZ(new_z);
    }
    last_pos = event->pos();
    update();
}

void OpenGLWidget::wheelEvent(QWheelEvent *event)
{
    fov -= event->angleDelta().y()/50.0f;
    if(fov<30.0f)
        fov = 30.0f;
    else if(fov>60.0f)
        fov = 60.0f;
    update();
}

void OpenGLWidget::updatePointCloudData()
{
    qDebug()<<"update";
    if(ctl_queue.empty())
        msg = ControlMsg::SENDCLOUD;
    else
    {
        msg = ctl_queue.front();
        ctl_queue.pop_front();
    }

    qDebug()<<msg;

    if(::send(sock,(char*)&msg,sizeof(msg),0)==-1)
    {
        closeConnect();
        return;
    }

    if(msg == ControlMsg::GETCONFIG || msg == ControlMsg::DEFAULTCONFIG)
    {
        if(::recv(sock,(char*)&cfg_msg,sizeof(cfg_msg),0)==-1)
        {
            closeConnect();
            return;
        }
        emit updateConfig(cfg_msg);
    }
    else if(msg == ControlMsg::SETCONFIG || msg == ControlMsg::SAVECONFIG)
    {
        if(::send(sock,(char*)&cfg_msg,sizeof(cfg_msg),0)==-1)
        {
            closeConnect();
            return;
        }
    }
    else if(msg == ControlMsg::SENDCLOUD)
    {
        bool ret = true;
        if(::recv(sock,(char*)&type,sizeof(type),0)==-1)
        {
            closeConnect();
            return;
        }
        if(::send(sock,(char*)&ret,sizeof(ret),0)==-1)
        {
            closeConnect();
            return;
        }
        if(!type.static_update&&!type.safe_update&&!type.warning_update)
            return;
        socket_datas[SAFE].len = socket_datas[WARNING].len = box_datas.len = 0;
        if(type.static_update)
        {
            // 获取点云数量
            if(::recv(sock,(char*)&socket_datas[STATIC].len,sizeof(unsigned int),0)==-1)
            {
                closeConnect();
                return;
            }
            if(::send(sock,(char*)&ret,sizeof(ret),0)==-1)
            {
                closeConnect();
                return;
            }
            // 计算分段数量
            unsigned int num = ceil((float)socket_datas[STATIC].len/(float)POINT_SEGMENT);
            unsigned int start = 0;
            for(unsigned int i=0;i<num;i++)
            {
                unsigned int len = std::min((unsigned int)POINT_SEGMENT,socket_datas[STATIC].len-start);
                if(::recv(sock,(char*)&socket_datas[STATIC].points[start],len*sizeof(Point),0)==-1)
                {
                    closeConnect();
                    return;
                }
                if(::send(sock,(char*)&ret,sizeof(ret),0)==-1)
                {
                    closeConnect();
                    return;
                }
                start += len;
            }
            glBindBuffer(GL_ARRAY_BUFFER,VBOs[STATIC]);
            glBufferData(GL_ARRAY_BUFFER,socket_datas[STATIC].len*sizeof(Point),socket_datas[STATIC].points,usages[STATIC]);
        }
        if(type.safe_update)
        {
            // 获取点云数量
            if(::recv(sock,(char*)&socket_datas[SAFE].len,sizeof(unsigned int),0)==-1)
            {
                closeConnect();
                return;
            }
            if(::send(sock,(char*)&ret,sizeof(ret),0)==-1)
            {
                closeConnect();
                return;
            }
            // 计算分段数量
            unsigned int num = ceil((float)socket_datas[SAFE].len/(float)POINT_SEGMENT);
            unsigned int start = 0;
            for(unsigned int i=0;i<num;i++)
            {
                unsigned int len = std::min((unsigned int)POINT_SEGMENT,socket_datas[SAFE].len-start);
                if(::recv(sock,(char*)&socket_datas[SAFE].points[start],len*sizeof(Point),0)==-1)
                {
                    closeConnect();
                    return;
                }
                if(::send(sock,(char*)&ret,sizeof(ret),0)==-1)
                {
                    closeConnect();
                    return;
                }
                start += len;
            }
            glBindBuffer(GL_ARRAY_BUFFER,VBOs[SAFE]);
            glBufferData(GL_ARRAY_BUFFER,socket_datas[SAFE].len*sizeof(Point),socket_datas[SAFE].points,usages[SAFE]);
        }
        if(type.warning_update)
        {
            // 获取点云数量
            if(::recv(sock,(char*)&socket_datas[WARNING].len,sizeof(unsigned int),0)==-1)
            {
                closeConnect();
                return;
            }
            if(::send(sock,(char*)&ret,sizeof(ret),0)==-1)
            {
                closeConnect();
                return;
            }
            // 计算分段数量
            unsigned int num = ceil((float)socket_datas[WARNING].len/(float)POINT_SEGMENT);
            unsigned int start = 0;
            for(unsigned int i=0;i<num;i++)
            {
                unsigned int len = std::min((unsigned int)POINT_SEGMENT,socket_datas[WARNING].len-start);
                if(::recv(sock,(char*)&socket_datas[WARNING].points[start],len*sizeof(Point),0)==-1)
                {
                    closeConnect();
                    return;
                }
                if(::send(sock,(char*)&ret,sizeof(ret),0)==-1)
                {
                    closeConnect();
                    return;
                }
                start += len;
            }
            // 提取点云尾部框图坐标
            if(socket_datas[WARNING].len>0)
            {
                unsigned int count = socket_datas[WARNING].points[socket_datas[WARNING].len-1].x; // 数量
                box_datas.len = count*2;
                socket_datas[WARNING].len -= count+1; // 加上尾部数量本身
                unsigned int index = socket_datas[WARNING].len;
                for(unsigned int i=0;i<count/2;i++)
                {
                    float min_x,min_y,max_x,max_y;
                    min_x = socket_datas[WARNING].points[index].x;
                    min_y = socket_datas[WARNING].points[index].y;
                    index++;
                    max_x = socket_datas[WARNING].points[index].x;
                    max_y = socket_datas[WARNING].points[index].y;
                    index++;

                    box_datas.points[i*4+0].x=min_x,box_datas.points[i*4+0].y=min_y;
                    box_datas.points[i*4+1].x=min_x,box_datas.points[i*4+1].y=max_y;
                    box_datas.points[i*4+2].x=max_x,box_datas.points[i*4+2].y=max_y;
                    box_datas.points[i*4+3].x=max_x,box_datas.points[i*4+3].y=min_y;
                }
            }

            glBindBuffer(GL_ARRAY_BUFFER,VBOs[WARNING]);
            glBufferData(GL_ARRAY_BUFFER,socket_datas[WARNING].len*sizeof(Point),socket_datas[WARNING].points,usages[WARNING]);
            glBindBuffer(GL_ARRAY_BUFFER,box_VBO);
            glBufferData(GL_ARRAY_BUFFER,box_datas.len*sizeof(Point),box_datas.points,GL_DYNAMIC_DRAW);
        }
        update();
    }
}

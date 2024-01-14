#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtOpenGL/QOpenGLFunctions_3_3_Core>
#include <QtOpenGL/QOpenGLShaderProgram>
#include <QMouseEvent>
#include <QTimer>
#include "socket_type.h"
#include <winsock2.h>
#include <QMessageBox>
#include <QQueue>

class OpenGLWidget : public QOpenGLWidget,QOpenGLFunctions_3_3_Core
{
    Q_OBJECT;
public:
    explicit OpenGLWidget(QWidget *patent = nullptr);
    ~OpenGLWidget();

    bool connectToServer(QString address,quint16 port);
    void closeConnect();

protected:
    virtual void initializeGL() override; // opengl初始化
    virtual void resizeGL(int w,int h) override; // opengl窗口大小修改
    virtual void paintGL() override; // opengl窗口绘制
    virtual void mousePressEvent(QMouseEvent *event) override; // 鼠标点击
    virtual void mouseMoveEvent(QMouseEvent *event) override; // 鼠标拖动
    virtual void wheelEvent(QWheelEvent *event) override; // 鼠标滚轮

private:
    // opengl变量
    unsigned int VAO_id,VBO_id;
    enum DATA_TYPE {STATIC=0,SAFE=1,WARNING=2};
    unsigned int VAOs[3];
    unsigned int VBOs[3];
    QOpenGLShaderProgram shaders[3]; // 渲染程序
    QVector3D cam_position; // 相机位置
    QVector3D cam_front; // 相机朝向
    QVector3D cam_up; // 相机上向量

    QOpenGLShaderProgram coordinate_shader; // 雷达图坐标渲染程序
    unsigned int coordinate_VAO;
    unsigned int coordinate_VBO;

    QOpenGLShaderProgram box_shader; // 框图渲染程序
    unsigned int box_VAO;
    unsigned int box_VBO;
    PointCloudData box_datas;

    float fov; // 视野
    float width,height;// 屏幕长、宽

    QPoint last_pos; // 鼠标坐标
    unsigned int usages[3] = {GL_STATIC_DRAW,GL_DYNAMIC_DRAW,GL_DYNAMIC_DRAW};

    // 网络变量
    SOCKET sock;
    UpdateDataType type; // 更新数据
    PointCloudData socket_datas[3]; // 点云数据
    ControlMsg msg; // 控制信息
    bool is_connected_;

    QTimer *timer;

public:
    // 显示check
    bool checks[3] = {true,true,true};
    bool box_check = true;
    QQueue<ControlMsg> ctl_queue;
    ConfigMsg cfg_msg;

signals:
    void updateConfig(ConfigMsg& cfg_msg);
public slots:
    void updatePointCloudData(); // tcp申请信息
};

#endif // OPENGLWIDGET_H

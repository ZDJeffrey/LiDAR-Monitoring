#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->IPEdit->setFocus();
    connect(ui->ConnectBottom,&QPushButton::clicked,[this](){
        QString ip_addr = ui->IPEdit->text();
        quint16 port = quint16(ui->PortEdit->text().toUInt());
        if(this->ui->openGLWidget->connectToServer(ip_addr,port))
            QMessageBox::information(this,"info","Connect to server successfully.");
        else
            QMessageBox::critical(this,"critical","Connect error.");
    });
    connect(ui->DisconnectBottom,&QPushButton::clicked,[this](){
        this->ui->openGLWidget->closeConnect();
    });

    connect(ui->StaticCheck,&QCheckBox::stateChanged,[&](int state){
        this->ui->openGLWidget->checks[0] = (bool)state;
    });
    connect(ui->SafeCheck,&QCheckBox::stateChanged,[&](int state){
        this->ui->openGLWidget->checks[1] = (bool)state;
    });
    connect(ui->WarningCheck,&QCheckBox::stateChanged,[&](int state){
        this->ui->openGLWidget->checks[2] = (bool)state;
    });
    connect(ui->BoxCheck,&QCheckBox::stateChanged,[&](int state){
        this->ui->openGLWidget->box_check = (bool)state;
    });
    connect(ui->StartButton,&QPushButton::clicked,[this](){
        this->ui->openGLWidget->ctl_queue.push_back(ControlMsg::START);
    });
    connect(ui->StopButton,&QPushButton::clicked,[this](){
        this->ui->openGLWidget->ctl_queue.push_back(ControlMsg::STOP);
    });
    connect(ui->InitializeButton,&QPushButton::clicked,[this](){
        this->ui->openGLWidget->ctl_queue.push_back(ControlMsg::INITIALIZATION);
    });
    connect(ui->setButton,&QPushButton::clicked,[this]{
        getConfig(this->ui->openGLWidget->cfg_msg);
        if(checkConfig(this->ui->openGLWidget->cfg_msg))
            this->ui->openGLWidget->ctl_queue.push_back(ControlMsg::SETCONFIG);
        else
            QMessageBox::critical(this,"critical","Illegal config.");
    });
    connect(ui->saveButton,&QPushButton::clicked,[this]{
        getConfig(this->ui->openGLWidget->cfg_msg);
        if(checkConfig(this->ui->openGLWidget->cfg_msg))
            this->ui->openGLWidget->ctl_queue.push_back(ControlMsg::SAVECONFIG);
        else
            QMessageBox::critical(this,"critical","Illegal config.");
    });
    connect(ui->defaultButton,&QPushButton::clicked,[this]{
        this->ui->openGLWidget->ctl_queue.push_back(ControlMsg::DEFAULTCONFIG);
    });
    connect(ui->openGLWidget,&OpenGLWidget::updateConfig,this,&MainWindow::showConfig);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showConfig(ConfigMsg& cfg_msg)
{
    switch(cfg_msg.mode_id)
    {
    case 0:
        ui->mode_0->setChecked(true);
        break;
    case 1:
        ui->mode_1->setChecked(true);
        break;
    case 2:
        ui->mode_2->setChecked(true);
        break;
    case 3:
        ui->mode_3->setChecked(true);
        break;
    case 4:
        ui->mode_4->setChecked(true);
        break;
    case 5:
        ui->mode_5->setChecked(true);
        break;
    }

    ui->freqEdit->setText(QString::number(cfg_msg.proc_freq));
    ui->maxDistEdit->setText(QString::number(cfg_msg.max_dist));
    ui->distEdit->setText(QString::number(cfg_msg.dist_threshold));
    ui->numEdit->setText(QString::number(cfg_msg.num_threshold));
    ui->volumeEdit->setText(QString::number(cfg_msg.alarm_volume));
}

bool MainWindow::checkConfig(ConfigMsg& cfg_msg)
{
    if(cfg_msg.mode_id>5) return false;
    if(cfg_msg.proc_freq<1||cfg_msg.proc_freq>15) return false;
    if(cfg_msg.max_dist<=0||cfg_msg.max_dist>16) return false;
    if(cfg_msg.num_threshold==0) return false;
    if(cfg_msg.alarm_volume>31) return false;
    return true;
}

void MainWindow::getConfig(ConfigMsg& cfg_msg)
{
    if(ui->mode_0->isChecked()) cfg_msg.mode_id = 0;
    else if(ui->mode_1->isChecked()) cfg_msg.mode_id = 1;
    else if(ui->mode_2->isChecked()) cfg_msg.mode_id = 2;
    else if(ui->mode_3->isChecked()) cfg_msg.mode_id = 3;
    else if(ui->mode_4->isChecked()) cfg_msg.mode_id = 4;
    else if(ui->mode_5->isChecked()) cfg_msg.mode_id = 5;

    cfg_msg.proc_freq = ui->freqEdit->text().toUInt();
    cfg_msg.max_dist = ui->maxDistEdit->text().toDouble();
    cfg_msg.dist_threshold = ui->distEdit->text().toDouble();
    cfg_msg.num_threshold = ui->numEdit->text().toUInt();
    cfg_msg.alarm_volume = ui->volumeEdit->text().toUInt();
}


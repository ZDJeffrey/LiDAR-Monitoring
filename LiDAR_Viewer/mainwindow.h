#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QCheckBox>
#include "socket_type.h"
#include "openglwidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class IPinput;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void showConfig(ConfigMsg& cfg_msg);
    bool checkConfig(ConfigMsg& cfg_msg);
    void getConfig(ConfigMsg& cfg_msg);
private:
    Ui::MainWindow *ui;

};
#endif // MAINWINDOW_H

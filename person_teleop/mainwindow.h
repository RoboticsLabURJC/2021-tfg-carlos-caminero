#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include "proxy.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_forward_button_clicked();

    void on_right_button_clicked();

    void on_left_button_clicked();

    void on_backward_button_clicked();

    void on_manual_control_button_clicked();

private:
    Ui::MainWindow *ui;

    void keyPressEvent(QKeyEvent *);
    bool manual_control_activated;

    int sockfd;
    struct sockaddr_in server_addr;
};
#endif // MAINWINDOW_H

#include <iostream>
#include <string>
#include "mainwindow.h"
#include "./ui_mainwindow.h"

const std::string SERVER_IP = "0.0.0.0";
const int PORT = 36677;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , manual_control_activated(false)
{
    ui->setupUi(this);
    QPixmap pixmap = QString("images/logo.png");
    ui->icon_label->setPixmap(pixmap.scaled(ui->icon_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    this->sockfd = create_socket();
    set_ip_port(this->server_addr, SERVER_IP.c_str(), PORT);
}

MainWindow::~MainWindow()
{
    close_socket(this->sockfd);
    delete ui;
}

void MainWindow::on_forward_button_clicked()
{
    if (!manual_control_activated) {
        return;
    }
    char msg[3] = {'U', 'V', 'F'};
    send_message(this->sockfd, this->server_addr, msg, sizeof msg);
}

void MainWindow::on_right_button_clicked()
{
    if (!manual_control_activated) {
        return;
    }
    char msg[3] = {'U', 'A', 'R'};
    send_message(this->sockfd, this->server_addr, msg, sizeof msg);
}

void MainWindow::on_left_button_clicked()
{
    if (!manual_control_activated) {
        return;
    }
    char msg[3] = {'U', 'A', 'L'};
    send_message(this->sockfd, this->server_addr, msg, sizeof msg);
}

void MainWindow::on_backward_button_clicked()
{
    if (!manual_control_activated) {
        return;
    }
    char msg[3] = {'U', 'V', 'B'};
    send_message(this->sockfd, this->server_addr, msg, sizeof msg);
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
    if (event->key() == Qt::Key_W) {
        on_forward_button_clicked();
    }
    else if (event->key() == Qt::Key_S) {
        on_backward_button_clicked();
    }
    else if (event->key() == Qt::Key_A) {
        on_left_button_clicked();
    }
    else if (event->key() == Qt::Key_D) {
        on_right_button_clicked();
    }
}

void MainWindow::on_manual_control_button_clicked()
{
    char msg[3] = {'U', 'S', '-'};
    if (ui->manual_control_button->isChecked()) {
        manual_control_activated = true;
    }
    else {
        manual_control_activated = false;
        msg[0] = 'A';
    }
    send_message(this->sockfd, this->server_addr, msg, sizeof msg);
}

#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QDateTime>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    socket = new QTcpSocket(this);

    ui->labelConnectStatus->setText("Disconnected");
    ui->labelConnectStatus->setStyleSheet("color:red; font-weight:bold;");

    connect(socket, &QTcpSocket::connected, this, [=](){
        ui->labelConnectStatus->setText("Connected");
        ui->labelConnectStatus->setStyleSheet("color:green; font-weight:bold;");
        addLog("Connected to ESP32");
    });

    connect(socket, &QTcpSocket::disconnected, this, [=](){
        ui->labelConnectStatus->setText("Disconnected");
        ui->labelConnectStatus->setStyleSheet("color:red; font-weight:bold;");
        addLog("Disconnected");
    });

    connect(socket, &QTcpSocket::readyRead, this, [=](){
        QString data = QString::fromUtf8(socket->readAll()).trimmed();
        addLog("ESP32: " + data);
    });

    connect(socket,
            QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
            this,
            [=](QAbstractSocket::SocketError){
                addLog("Error: " + socket->errorString());
            });

    connect(ui->sliderR, &QSlider::valueChanged, this, &MainWindow::sendRGB);
    connect(ui->sliderG, &QSlider::valueChanged, this, &MainWindow::sendRGB);
    connect(ui->sliderB, &QSlider::valueChanged, this, &MainWindow::sendRGB);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addLog(const QString &msg)
{
    QString time = QDateTime::currentDateTime().toString("hh:mm:ss");
    ui->textBrowserLog->append(QString("[%1] %2").arg(time).arg(msg));
}

void MainWindow::on_btnConnect_clicked()
{
    QString ip = ui->lineEditIP->text();
    int port = ui->lineEditPort->text().toInt();

    addLog(QString("Connecting to %1:%2").arg(ip).arg(port));
    socket->connectToHost(ip, port);
}

void MainWindow::on_btnLedOn_clicked()
{
    if(socket->state() == QAbstractSocket::ConnectedState)
    {
        ledOn = true;
        socket->write("LED_ON\n");
        addLog("Send: LED_ON");
    }
}

void MainWindow::on_btnLedOff_clicked()
{
    ledOn = false;

    if(socket->state() == QAbstractSocket::ConnectedState)
    {
        socket->write("LED_OFF\n");
        addLog("Send: LED_OFF");
    }

    ui->colorPreview->setStyleSheet(
        "background-color: black;"
        "border: 2px solid gray;"
        );
}

void MainWindow::sendRGB()
{
    int r = ui->sliderR->value();
    int g = ui->sliderG->value();
    int b = ui->sliderB->value();

    ui->colorPreview->setStyleSheet(
        QString(
            "background-color: rgb(%1,%2,%3);"
            "border: 2px solid gray;"
            )
            .arg(r)
            .arg(g)
            .arg(b)
        );

    if(!ledOn)
        return;

    if(socket->state() != QAbstractSocket::ConnectedState)
        return;

    QString cmd = QString("RGB:%1,%2,%3\n")
                      .arg(r)
                      .arg(g)
                      .arg(b);

    socket->write(cmd.toUtf8());
    addLog("Send: " + cmd.trimmed());
}
#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QDebug>
#include <QDateTime>
#include <QTimer>
#include <QRegularExpression>
#include <QDoubleValidator>

// =====================================================
// Constructor
// =====================================================
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    socket = new QTcpSocket(this);

    // =====================================================
    // 默认 IP 和端口
    // =====================================================
    ui->lineEditIP->setText("192.168.4.1");
    ui->lineEditPort->setText("8080");

    ui->labelConnectStatus->setText("Disconnected");
    ui->labelConnectStatus->setStyleSheet("color:red; font-weight:bold;");

    // =====================================================
    // 默认参数显示
    // =====================================================
    ui->kpEdit->setText("5.0");
    ui->kiEdit->setText("0.015");


    ui->kdEdit->setText("0.0");

    ui->maxhzEdit->setText("550");
    ui->minhzEdit->setText("140");
    ui->accelEdit->setText("4500");

    ui->CPREdit->setText("40");
    ui->toleranceEdit->setText("4.5");

    ui->currentAngleEdit->setReadOnly(true);

    // =====================================================
    // 输入限制
    // =====================================================
    ui->angleEdit->setValidator(new QDoubleValidator(-64800.0, 64800.0, 3, this));

    ui->kpEdit->setValidator(new QDoubleValidator(0.0, 30.0, 4, this));
    ui->kiEdit->setValidator(new QDoubleValidator(0.0, 1.0, 6, this));
    ui->kdEdit->setValidator(new QDoubleValidator(0.0, 5.0, 6, this));

    ui->maxhzEdit->setValidator(new QDoubleValidator(1.0, 3000.0, 2, this));
    ui->minhzEdit->setValidator(new QDoubleValidator(1.0, 3000.0, 2, this));
    ui->accelEdit->setValidator(new QDoubleValidator(1.0, 30000.0, 2, this));

    ui->CPREdit->setValidator(new QDoubleValidator(1.0, 10000.0, 4, this));
    ui->toleranceEdit->setValidator(new QDoubleValidator(0.1, 30.0, 3, this));

    // =====================================================
    // 定时读取当前角度
    // =====================================================
    QTimer *angleTimer = new QTimer(this);
    angleTimer->setInterval(1000);

    connect(angleTimer, &QTimer::timeout, this, [=]() {
        if (socket->state() == QAbstractSocket::ConnectedState) {
            sendCommand("ANGLE?", false);
        }
    });

    // =====================================================
    // Socket signals
    // =====================================================
    connect(socket, &QTcpSocket::connected, this, [=]() {
        ui->labelConnectStatus->setText("Connected");
        ui->labelConnectStatus->setStyleSheet("color:green; font-weight:bold;");
        ui->btnConnect->setText("Disconnect");

        addLog("Connected to ESP32");

        // 连接后自动读取 Flash 里的配置
        QTimer::singleShot(200, this, [=]() {
            sendCommand("CFG?", true);
        });

        angleTimer->start();
    });

    connect(socket, &QTcpSocket::disconnected, this, [=]() {
        ui->labelConnectStatus->setText("Disconnected");
        ui->labelConnectStatus->setStyleSheet("color:red; font-weight:bold;");
        ui->btnConnect->setText("Connect");

        addLog("Disconnected");

        angleTimer->stop();
    });

    connect(socket, &QTcpSocket::readyRead, this, [=]() {
        rxBuffer += QString::fromUtf8(socket->readAll());

        while (rxBuffer.contains('\n')) {
            int index = rxBuffer.indexOf('\n');

            QString line = rxBuffer.left(index).trimmed();
            rxBuffer.remove(0, index + 1);

            if (!line.isEmpty()) {
                addLog("ESP32: " + line);
                handleEsp32Line(line);
            }
        }
    });

    connect(socket, &QTcpSocket::errorOccurred, this, [=](QAbstractSocket::SocketError) {
        addLog("Error: " + socket->errorString());
    });

    // =====================================================
    // RGB sliders
    // =====================================================
    connect(ui->sliderR, &QSlider::valueChanged, this, &MainWindow::sendRGB);
    connect(ui->sliderG, &QSlider::valueChanged, this, &MainWindow::sendRGB);
    connect(ui->sliderB, &QSlider::valueChanged, this, &MainWindow::sendRGB);

    // =====================================================
    // 输入框按 Enter 直接发送参数
    // =====================================================
    connect(ui->kpEdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplyPid_clicked);
    connect(ui->kiEdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplyPid_clicked);
    connect(ui->kdEdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplyPid_clicked);

    connect(ui->maxhzEdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplySpeed_clicked);
    connect(ui->minhzEdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplySpeed_clicked);
    connect(ui->accelEdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplySpeed_clicked);

    connect(ui->CPREdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplyEncoder_clicked);
    connect(ui->toleranceEdit, &QLineEdit::returnPressed, this, &MainWindow::on_btnApplyTolerance_clicked);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// =====================================================
// Log
// =====================================================
void MainWindow::addLog(const QString &msg)
{
    QString time = QDateTime::currentDateTime().toString("hh:mm:ss");
    ui->textBrowserLog->append(QString("[%1] %2").arg(time).arg(msg));
}

// =====================================================
// Send helper
// =====================================================
void MainWindow::sendCommand(const QString &cmd, bool showLog)
{
    if (socket->state() != QAbstractSocket::ConnectedState) {
        addLog("Not connected");
        return;
    }

    QString line = cmd.trimmed() + "\n";
    socket->write(line.toUtf8());

    if (showLog) {
        addLog("Send: " + cmd.trimmed());
    }
}

// =====================================================
// Parse ESP32 reply
// =====================================================
QString MainWindow::valueFromLine(const QString &line, const QString &key) const
{
    QRegularExpression re(
        "\\b" + QRegularExpression::escape(key) + "=([-+]?\\d+(?:\\.\\d+)?)"
        );

    QRegularExpressionMatch match = re.match(line);

    if (match.hasMatch()) {
        return match.captured(1);
    }

    return "";
}

void MainWindow::updateConfigEditsFromLine(const QString &line)
{
    QString v;

    v = valueFromLine(line, "KP");
    if (!v.isEmpty()) {
        ui->kpEdit->setText(v);
    }

    v = valueFromLine(line, "KI");
    if (!v.isEmpty()) {
        ui->kiEdit->setText(v);
    }

    v = valueFromLine(line, "KD");
    if (!v.isEmpty()) {
        ui->kdEdit->setText(v);
    }

    v = valueFromLine(line, "MAXHZ");
    if (v.isEmpty()) {
        v = valueFromLine(line, "MAX");
    }
    if (!v.isEmpty()) {
        ui->maxhzEdit->setText(v);
    }

    v = valueFromLine(line, "MINHZ");
    if (v.isEmpty()) {
        v = valueFromLine(line, "MIN");
    }
    if (!v.isEmpty()) {
        ui->minhzEdit->setText(v);
    }

    v = valueFromLine(line, "ACCEL");
    if (!v.isEmpty()) {
        ui->accelEdit->setText(v);
    }

    v = valueFromLine(line, "CPR");
    if (!v.isEmpty()) {
        ui->CPREdit->setText(v);
    }

    v = valueFromLine(line, "TOL");
    if (!v.isEmpty()) {
        ui->toleranceEdit->setText(v);
    }
}

void MainWindow::handleEsp32Line(const QString &line)
{
    // CFG KP=... KI=... KD=... MAXHZ=... MINHZ=... ACCEL=... CPR=... TOL=...
    if (line.startsWith("CFG ") ||
        line.startsWith("OK CFG_SAVE") ||
        line.startsWith("OK CFG_RESET")) {
        updateConfigEditsFromLine(line);
        return;
    }

    // PID KP=...
    if (line.startsWith("PID ") || line.startsWith("OK PID")) {
        updateConfigEditsFromLine(line);
        return;
    }

    // SPEED MAX=...
    if (line.startsWith("SPEED ") || line.startsWith("OK SPEED")) {
        updateConfigEditsFromLine(line);
        return;
    }

    // ENCODER CPR=...
    if (line.startsWith("ENCODER ") || line.startsWith("OK ENCODER")) {
        updateConfigEditsFromLine(line);
        return;
    }

    // TOL=...
    if (line.startsWith("TOL=") || line.startsWith("OK TOL")) {
        updateConfigEditsFromLine(line);
        return;
    }

    // ANGLE CURRENT=90.00 ENCODER=10
    if (line.startsWith("ANGLE CURRENT=")) {
        QString angle = valueFromLine(line, "CURRENT");

        if (!angle.isEmpty()) {
            ui->currentAngleEdit->setText(angle);
        }

        return;
    }
}

// =====================================================
// Connect button
// =====================================================
void MainWindow::on_btnConnect_clicked()
{
    if (socket->state() == QAbstractSocket::ConnectedState) {
        socket->disconnectFromHost();
        return;
    }

    QString ip = ui->lineEditIP->text().trimmed();
    int port = ui->lineEditPort->text().toInt();

    addLog(QString("Connecting to %1:%2").arg(ip).arg(port));

    socket->abort();
    socket->connectToHost(ip, port);
}

// =====================================================
// LED
// =====================================================
void MainWindow::on_btnLedOn_clicked()
{
    ledOn = true;
    sendCommand("LED_ON");
}

void MainWindow::on_btnLedOff_clicked()
{
    ledOn = false;

    sendCommand("LED_OFF");

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

    if (!ledOn) {
        return;
    }

    QString cmd = QString("RGB:%1,%2,%3")
                      .arg(r)
                      .arg(g)
                      .arg(b);

    sendCommand(cmd);
}

// =====================================================
// Angle
// =====================================================
void MainWindow::on_sendAngleButton_clicked()
{
    QString angle = ui->angleEdit->text().trimmed();

    if (angle.isEmpty()) {
        return;
    }

    QString cmd = QString("ANGLE:%1").arg(angle);
    sendCommand(cmd);
}

// =====================================================
// Apply PID
// 命令：PID:Kp,Ki,Kd
// =====================================================
void MainWindow::on_btnApplyPid_clicked()
{
    QString kp = ui->kpEdit->text().trimmed();
    QString ki = ui->kiEdit->text().trimmed();
    QString kd = ui->kdEdit->text().trimmed();

    if (kp.isEmpty() || ki.isEmpty() || kd.isEmpty()) {
        addLog("PID input empty");
        return;
    }

    QString cmd = QString("PID:%1,%2,%3")
                      .arg(kp)
                      .arg(ki)
                      .arg(kd);

    sendCommand(cmd);
}

// =====================================================
// Apply Speed
// 命令：SPEED:maxHz,minHz,accel
// =====================================================
void MainWindow::on_btnApplySpeed_clicked()
{
    QString maxHz = ui->maxhzEdit->text().trimmed();
    QString minHz = ui->minhzEdit->text().trimmed();
    QString accel = ui->accelEdit->text().trimmed();

    if (maxHz.isEmpty() || minHz.isEmpty() || accel.isEmpty()) {
        addLog("Speed input empty");
        return;
    }

    QString cmd = QString("SPEED:%1,%2,%3")
                      .arg(maxHz)
                      .arg(minHz)
                      .arg(accel);

    sendCommand(cmd);
}

// =====================================================
// Apply Encoder CPR
// 命令：ENCODER:40
// =====================================================
void MainWindow::on_btnApplyEncoder_clicked()
{
    QString cpr = ui->CPREdit->text().trimmed();

    if (cpr.isEmpty()) {
        addLog("Encoder CPR input empty");
        return;
    }

    QString cmd = QString("ENCODER:%1").arg(cpr);
    sendCommand(cmd);
}

// =====================================================
// Apply Tolerance
// 命令：TOL:4.5
// =====================================================
void MainWindow::on_btnApplyTolerance_clicked()
{
    QString tol = ui->toleranceEdit->text().trimmed();

    if (tol.isEmpty()) {
        addLog("Tolerance input empty");
        return;
    }

    QString cmd = QString("TOL:%1").arg(tol);
    sendCommand(cmd);
}

// =====================================================
// Read Config
// =====================================================
void MainWindow::on_btnReadConfig_clicked()
{
    sendCommand("CFG?");
}

// =====================================================
// Save Flash
// =====================================================
void MainWindow::on_btnSaveConfig_clicked()
{
    sendCommand("CFG_SAVE");
}

// =====================================================
// Reset Config
// =====================================================
void MainWindow::on_btnResetConfig_clicked()
{
    sendCommand("CFG_RESET");
}


void MainWindow::on_btnSendParameter_clicked()
{
    QString kp = ui->kpEdit->text().trimmed();
    QString ki = ui->kiEdit->text().trimmed();
    QString kd = ui->kdEdit->text().trimmed();   // 如果你改名 kdEdit，这里换成 ui->kdEdit

    QString maxHz = ui->maxhzEdit->text().trimmed();
    QString minHz = ui->minhzEdit->text().trimmed();
    QString accel = ui->accelEdit->text().trimmed();

    QString cpr = ui->CPREdit->text().trimmed();
    QString tol = ui->toleranceEdit->text().trimmed();

    if (kp.isEmpty() || ki.isEmpty() || kd.isEmpty() ||
        maxHz.isEmpty() || minHz.isEmpty() || accel.isEmpty() ||
        cpr.isEmpty() || tol.isEmpty()) {
        addLog("Parameter input empty");
        return;
    }

    bool okKp = false;
    bool okKi = false;
    bool okKd = false;
    bool okMax = false;
    bool okMin = false;
    bool okAccel = false;
    bool okCpr = false;
    bool okTol = false;

    double kpVal = kp.toDouble(&okKp);
    double kiVal = ki.toDouble(&okKi);
    double kdVal = kd.toDouble(&okKd);
    double maxVal = maxHz.toDouble(&okMax);
    double minVal = minHz.toDouble(&okMin);
    double accelVal = accel.toDouble(&okAccel);
    double cprVal = cpr.toDouble(&okCpr);
    double tolVal = tol.toDouble(&okTol);

    if (!okKp || !okKi || !okKd ||
        !okMax || !okMin || !okAccel ||
        !okCpr || !okTol) {
        addLog("Parameter format error");
        return;
    }

    if (kpVal < 0 || kpVal > 30 ||
        kiVal < 0 || kiVal > 1 ||
        kdVal < 0 || kdVal > 5) {
        addLog("PID range error");
        return;
    }

    if (maxVal < 1 || maxVal > 3000 ||
        minVal < 1 || minVal > maxVal ||
        accelVal < 1 || accelVal > 30000) {
        addLog("Speed range error: check Max Hz / Min Hz / Accel");
        return;
    }

    if (cprVal < 1 || cprVal > 10000) {
        addLog("Encoder CPR range error");
        return;
    }

    if (tolVal < 0.1 || tolVal > 30) {
        addLog("Tolerance range error");
        return;
    }

    sendCommand(QString("PID:%1,%2,%3").arg(kp).arg(ki).arg(kd));
    sendCommand(QString("SPEED:%1,%2,%3").arg(maxHz).arg(minHz).arg(accel));
    sendCommand(QString("ENCODER:%1").arg(cpr));
    sendCommand(QString("TOL:%1").arg(tol));

    addLog("All parameters sent to ESP32 RAM, not Flash");
}
#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QDebug>
#include <QDateTime>
#include <QTimer>
#include <QRegularExpression>
#include <QDoubleValidator>
#include <QComboBox>
#include <QLabel>
#include <QLayout>
#include <QBoxLayout>
#include <QGridLayout>

// =====================================================
// Motor send-angle state
// 上位机发出 ANGLE 后，必须等 ESP32 返回 MOTOR_DONE 才能再次发送角度
// 角度发送模式下拉框：
//   0 = 单次发送角度
//   1 = 来回旋转：+A, -A, +A, -A ... 无限循环
//
// 来回旋转模式下：
//   - 按钮保持可点击，显示“停止来回旋转”
//   - 点击停止后，不会立刻打断当前电机动作
//   - 等当前动作收到 MOTOR_DONE 后，不再发送下一条 ANGLE
// =====================================================
namespace {
enum AngleSendMode {
    MODE_SINGLE = 0,
    MODE_BACK_AND_FORTH = 1
};

bool gMotorMoving = false;
bool gAngleSequenceRunning = false;
bool gAngleSequenceStopRequested = false;
double gBackAndForthBaseAngle = 0.0;
int gBackAndForthSentCount = 0;   // 已经发送了多少段来回旋转命令

QComboBox *gAngleModeCombo = nullptr;
QLabel *gAngleModeLabel = nullptr;

const char *SEND_ANGLE_SINGLE_IDLE_STYLE =
    "QPushButton {"
    " background-color: #2ecc71;"
    " color: white;"
    " font-weight: bold;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:hover {"
    " background-color: #27ae60;"
    "}";

const char *SEND_ANGLE_SWING_IDLE_STYLE =
    "QPushButton {"
    " background-color: #3498db;"
    " color: white;"
    " font-weight: bold;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:hover {"
    " background-color: #2980b9;"
    "}";

const char *SEND_ANGLE_BUSY_STYLE =
    "QPushButton {"
    " background-color: #f39c12;"
    " color: white;"
    " font-weight: bold;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:disabled {"
    " background-color: #f39c12;"
    " color: white;"
    " font-weight: bold;"
    "}";

const char *SEND_ANGLE_SWING_BUSY_STYLE =
    "QPushButton {"
    " background-color: #e74c3c;"
    " color: white;"
    " font-weight: bold;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:hover {"
    " background-color: #c0392b;"
    "}";

const char *SEND_ANGLE_SWING_STOPPING_STYLE =
    "QPushButton {"
    " background-color: #7f8c8d;"
    " color: white;"
    " font-weight: bold;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:disabled {"
    " background-color: #7f8c8d;"
    " color: white;"
    " font-weight: bold;"
    "}";

int currentAngleSendMode()
{
    if (gAngleModeCombo == nullptr) {
        return MODE_SINGLE;
    }

    return gAngleModeCombo->currentIndex();
}

QString formatAngleForCommand(double angle)
{
    // ESP32 可以接收普通数字，这里保留 3 位小数，去掉多余的 0，日志更清晰。
    QString text = QString::number(angle, 'f', 3);

    while (text.contains('.') && text.endsWith('0')) {
        text.chop(1);
    }

    if (text.endsWith('.')) {
        text.chop(1);
    }

    if (text == "-0") {
        text = "0";
    }

    return text;
}

double nextBackAndForthAngle()
{
    // 第 1 次 +A，第 2 次 -A，第 3 次 +A ... 无限交替。
    if ((gBackAndForthSentCount % 2) == 0) {
        return gBackAndForthBaseAngle;
    }

    return -gBackAndForthBaseAngle;
}

void setAngleModeWidgetsEnabled(bool enabled)
{
    if (gAngleModeCombo != nullptr) {
        gAngleModeCombo->setEnabled(enabled);
    }

    if (gAngleModeLabel != nullptr) {
        gAngleModeLabel->setEnabled(enabled);
    }
}

void setSendAngleIdle(Ui::MainWindow *ui)
{
    ui->sendAngleButton->setEnabled(true);
    ui->angleEdit->setEnabled(true);
    setAngleModeWidgetsEnabled(true);

    if (currentAngleSendMode() == MODE_BACK_AND_FORTH) {
        ui->sendAngleButton->setText("开始来回旋转");
        ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SWING_IDLE_STYLE);
    } else {
        ui->sendAngleButton->setText("发送角度");
        ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SINGLE_IDLE_STYLE);
    }
}

void setSendAngleBusy(Ui::MainWindow *ui)
{
    ui->angleEdit->setEnabled(false);
    setAngleModeWidgetsEnabled(false);

    if (gAngleSequenceRunning) {
        if (gAngleSequenceStopRequested) {
            ui->sendAngleButton->setEnabled(false);
            ui->sendAngleButton->setText("停止中，等待本次完成...");
            ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SWING_STOPPING_STYLE);
        } else {
            // 来回旋转时按钮保持可点击，用来请求停止；不会立即打断当前动作。
            ui->sendAngleButton->setEnabled(true);
            ui->sendAngleButton->setText(
                QString("停止来回旋转（第%1次）").arg(gBackAndForthSentCount)
                );
            ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SWING_BUSY_STYLE);
        }
    } else {
        ui->sendAngleButton->setEnabled(false);
        ui->sendAngleButton->setText("电机转动中...");
        ui->sendAngleButton->setStyleSheet(SEND_ANGLE_BUSY_STYLE);
    }
}

void setupAngleModeCombo(Ui::MainWindow *ui)
{
    QWidget *parent = ui->sendAngleButton->parentWidget();

    if (parent == nullptr) {
        return;
    }

    // 如果以后你在 Qt Designer 里自己放了 objectName=angleModeCombo 的下拉框，
    // 这里会直接复用，不会重复创建。
    gAngleModeCombo = parent->findChild<QComboBox *>("angleModeCombo");
    gAngleModeLabel = parent->findChild<QLabel *>("angleModeLabel");

    if (gAngleModeCombo == nullptr) {
        gAngleModeLabel = new QLabel("角度模式:", parent);
        gAngleModeLabel->setObjectName("angleModeLabel");

        gAngleModeCombo = new QComboBox(parent);
        gAngleModeCombo->setObjectName("angleModeCombo");
        gAngleModeCombo->addItem("单次发送角度");
        gAngleModeCombo->addItem("来回旋转：+A -A 无限循环");

        QLayout *layout = parent->layout();

        if (QBoxLayout *boxLayout = qobject_cast<QBoxLayout *>(layout)) {
            int buttonIndex = boxLayout->indexOf(ui->sendAngleButton);
            if (buttonIndex < 0) {
                buttonIndex = boxLayout->count();
            }

            boxLayout->insertWidget(buttonIndex, gAngleModeLabel);
            boxLayout->insertWidget(buttonIndex + 1, gAngleModeCombo);
        } else if (QGridLayout *gridLayout = qobject_cast<QGridLayout *>(layout)) {
            int row = 0;
            int column = 0;
            int rowSpan = 1;
            int columnSpan = 1;

            int buttonIndex = gridLayout->indexOf(ui->sendAngleButton);
            if (buttonIndex >= 0) {
                gridLayout->getItemPosition(buttonIndex, &row, &column, &rowSpan, &columnSpan);
                gridLayout->addWidget(gAngleModeLabel, row + 1, column, 1, 1);
                gridLayout->addWidget(gAngleModeCombo, row + 1, column + 1, 1, columnSpan);
            } else {
                gridLayout->addWidget(gAngleModeLabel, gridLayout->rowCount(), 0, 1, 1);
                gridLayout->addWidget(gAngleModeCombo, gridLayout->rowCount() - 1, 1, 1, 2);
            }
        } else {
            // 没有布局时兜底显示在按钮下方。
            const QRect buttonRect = ui->sendAngleButton->geometry();
            gAngleModeLabel->setGeometry(buttonRect.left(), buttonRect.bottom() + 8, 70, 24);
            gAngleModeCombo->setGeometry(buttonRect.left() + 75, buttonRect.bottom() + 8, 200, 24);
            gAngleModeLabel->show();
            gAngleModeCombo->show();
        }
    }

    QObject::connect(
        gAngleModeCombo,
        QOverload<int>::of(&QComboBox::currentIndexChanged),
        ui->sendAngleButton,
        [=](int) {
            if (!gMotorMoving && !gAngleSequenceRunning) {
                setSendAngleIdle(ui);
            }
        }
        );
}
}

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
    // PID + 单 Speed 参数
    // =====================================================
    ui->kpEdit->setText("5.0");
    ui->kiEdit->setText("0.015");
    ui->kdEdit->setText("0.0");

    ui->speedEdit->setText("500");

    ui->CPREdit->setText("40");
    ui->toleranceEdit->setText("4.5");

    ui->currentAngleEdit->setReadOnly(true);

    // 角度发送模式下拉框 + 按钮初始状态
    setupAngleModeCombo(ui);
    gMotorMoving = false;
    gAngleSequenceRunning = false;
    gAngleSequenceStopRequested = false;
    gBackAndForthBaseAngle = 0.0;
    gBackAndForthSentCount = 0;
    setSendAngleIdle(ui);

    // =====================================================
    // 输入限制
    // =====================================================
    ui->angleEdit->setValidator(
        new QDoubleValidator(-64800.0, 64800.0, 3, this)
        );

    ui->kpEdit->setValidator(
        new QDoubleValidator(0.0, 100.0, 4, this)
        );

    ui->kiEdit->setValidator(
        new QDoubleValidator(0.0, 10.0, 6, this)
        );

    ui->kdEdit->setValidator(
        new QDoubleValidator(0.0, 10.0, 6, this)
        );

    ui->speedEdit->setValidator(
        new QDoubleValidator(1.0, 3000.0, 2, this)
        );

    ui->CPREdit->setValidator(
        new QDoubleValidator(1.0, 10000.0, 4, this)
        );

    ui->toleranceEdit->setValidator(
        new QDoubleValidator(0.1, 30.0, 3, this)
        );

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

        // 连接后自动读取 ESP32 当前配置
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

        // 断开连接时解除电机忙状态，避免按钮一直锁住
        gMotorMoving = false;
        gAngleSequenceRunning = false;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = 0.0;
        gBackAndForthSentCount = 0;
        setSendAngleIdle(ui);

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

    v = valueFromLine(line, "SPEED");
    if (!v.isEmpty()) {
        ui->speedEdit->setText(v);
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
    // ESP32 在电机真正停止后会主动发送：MOTOR_DONE STATUS=OK ...
    // 单次模式：收到 MOTOR_DONE 后解锁按钮。
    // 来回旋转模式：收到 MOTOR_DONE 后，如果没有请求停止，就继续发送下一段；
    // 如果已经按过“停止来回旋转”，则在本次 MOTOR_DONE 后结束，不再发送下一条。
    if (line.startsWith("MOTOR_DONE")) {
        QString finalAngle = valueFromLine(line, "FINAL");
        if (!finalAngle.isEmpty()) {
            ui->currentAngleEdit->setText(finalAngle);
        }

        gMotorMoving = false;

        if (gAngleSequenceRunning) {
            if (gAngleSequenceStopRequested) {
                gAngleSequenceRunning = false;
                gAngleSequenceStopRequested = false;
                gBackAndForthBaseAngle = 0.0;
                gBackAndForthSentCount = 0;
                setSendAngleIdle(ui);
                addLog("Back-and-forth stopped after current MOTOR_DONE");
                return;
            }

            const double nextAngle = nextBackAndForthAngle();
            gBackAndForthSentCount++;

            gMotorMoving = true;
            setSendAngleBusy(ui);

            addLog(
                QString("Back-and-forth next angle #%1: %2")
                    .arg(gBackAndForthSentCount)
                    .arg(formatAngleForCommand(nextAngle))
                );

            // 稍微延迟 100ms 再发下一段，让 ESP32 的 MOTOR_DONE 和上位机日志先处理完。
            QTimer::singleShot(100, this, [=]() {
                if (!gAngleSequenceRunning) {
                    return;
                }

                if (gAngleSequenceStopRequested) {
                    gMotorMoving = false;
                    gAngleSequenceRunning = false;
                    gAngleSequenceStopRequested = false;
                    gBackAndForthBaseAngle = 0.0;
                    gBackAndForthSentCount = 0;
                    setSendAngleIdle(ui);
                    addLog("Back-and-forth stopped before sending next angle");
                    return;
                }

                if (socket->state() != QAbstractSocket::ConnectedState) {
                    addLog("Not connected, back-and-forth stopped");
                    gMotorMoving = false;
                    gAngleSequenceRunning = false;
                    gAngleSequenceStopRequested = false;
                    gBackAndForthBaseAngle = 0.0;
                    gBackAndForthSentCount = 0;
                    setSendAngleIdle(ui);
                    return;
                }

                sendCommand(QString("ANGLE:%1").arg(formatAngleForCommand(nextAngle)));
            });

            return;
        }

        setSendAngleIdle(ui);
        addLog("Motor finished, send angle unlocked");
        return;
    }

    // 如果 ANGLE 命令被 ESP32 拒绝，不会再有 MOTOR_DONE，这里解除锁定。
    if (gMotorMoving &&
        (line.startsWith("ERR") || line.startsWith("UNKNOWN CMD"))) {
        gMotorMoving = false;
        gAngleSequenceRunning = false;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = 0.0;
        gBackAndForthSentCount = 0;
        setSendAngleIdle(ui);
        addLog("Motor command failed, send angle unlocked");
        return;
    }
    // CFG KP=... KI=... KD=... SPEED=... CPR=... TOL=...
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

    // SPEED=... / OK SPEED=...
    if (line.startsWith("SPEED") || line.startsWith("OK SPEED")) {
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
    // 来回旋转正在运行时，按钮用于“请求停止”。
    // 不会立刻打断当前电机动作，而是等 ESP32 返回本次 MOTOR_DONE 后停止发送下一条。
    if (gAngleSequenceRunning) {
        if (!gAngleSequenceStopRequested) {
            gAngleSequenceStopRequested = true;
            setSendAngleBusy(ui);
            addLog("Stop requested: wait for current MOTOR_DONE, then no next angle will be sent");
        }
        return;
    }

    if (gMotorMoving) {
        addLog("Motor is moving, wait for MOTOR_DONE before sending next angle");
        return;
    }

    if (socket->state() != QAbstractSocket::ConnectedState) {
        addLog("Not connected");
        return;
    }

    QString angleText = ui->angleEdit->text().trimmed();

    if (angleText.isEmpty()) {
        return;
    }

    bool ok = false;
    double angleValue = angleText.toDouble(&ok);

    if (!ok) {
        addLog("Angle format error");
        return;
    }

    double firstAngle = angleValue;

    if (currentAngleSendMode() == MODE_BACK_AND_FORTH) {
        // 来回旋转模式：+A, -A, +A, -A ... 无限循环。
        // 每一段都必须等 ESP32 返回 MOTOR_DONE 后才会发下一段。
        gAngleSequenceRunning = true;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = angleValue;
        gBackAndForthSentCount = 1;
        firstAngle = angleValue;

        addLog(
            QString("Back-and-forth infinite start: first=%1, then alternate %2 / %3")
                .arg(formatAngleForCommand(firstAngle))
                .arg(formatAngleForCommand(angleValue))
                .arg(formatAngleForCommand(-angleValue))
            );
    } else {
        // 单次发送角度模式。
        gAngleSequenceRunning = false;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = 0.0;
        gBackAndForthSentCount = 0;
    }

    // 先锁定状态：单次模式收到 MOTOR_DONE 后解除；
    // 来回旋转模式收到 MOTOR_DONE 后继续发下一段，直到按下停止按钮。
    gMotorMoving = true;
    setSendAngleBusy(ui);

    QString cmd = QString("ANGLE:%1").arg(formatAngleForCommand(firstAngle));
    sendCommand(cmd);
}

// =====================================================
// Save Parameter
// 一个按钮：发送全部参数到 ESP32 RAM，然后保存到 Flash
// 命令：
// PID:Kp,Ki,Kd
// SPEED:Speed
// ENCODER:CPR
// TOL:Tolerance
// CFG_SAVE
// =====================================================
void MainWindow::on_btnSaveParameter_clicked()
{
    QString kp = ui->kpEdit->text().trimmed();
    QString ki = ui->kiEdit->text().trimmed();
    QString kd = ui->kdEdit->text().trimmed();

    QString speed = ui->speedEdit->text().trimmed();

    QString cpr = ui->CPREdit->text().trimmed();
    QString tol = ui->toleranceEdit->text().trimmed();

    if (kp.isEmpty() || ki.isEmpty() || kd.isEmpty() ||
        speed.isEmpty() || cpr.isEmpty() || tol.isEmpty()) {
        addLog("Parameter input empty");
        return;
    }

    bool okKp = false;
    bool okKi = false;
    bool okKd = false;
    bool okSpeed = false;
    bool okCpr = false;
    bool okTol = false;

    double kpVal = kp.toDouble(&okKp);
    double kiVal = ki.toDouble(&okKi);
    double kdVal = kd.toDouble(&okKd);
    double speedVal = speed.toDouble(&okSpeed);
    double cprVal = cpr.toDouble(&okCpr);
    double tolVal = tol.toDouble(&okTol);

    if (!okKp || !okKi || !okKd ||
        !okSpeed || !okCpr || !okTol) {
        addLog("Parameter format error");
        return;
    }

    if (kpVal < 0 || kpVal > 100 ||
        kiVal < 0 || kiVal > 10 ||
        kdVal < 0 || kdVal > 10) {
        addLog("PID range error");
        return;
    }

    if (speedVal < 1 || speedVal > 3000) {
        addLog("Speed range error");
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

    // 1. 先把界面参数发到 ESP32 RAM
    sendCommand(QString("PID:%1,%2,%3").arg(kp).arg(ki).arg(kd));
    sendCommand(QString("SPEED:%1").arg(speed));
    sendCommand(QString("ENCODER:%1").arg(cpr));
    sendCommand(QString("TOL:%1").arg(tol));

    // 2. 再保存 ESP32 当前 RAM 参数到 Flash
    sendCommand("CFG_SAVE");

    addLog("All parameters sent and saved to ESP32 Flash");
}

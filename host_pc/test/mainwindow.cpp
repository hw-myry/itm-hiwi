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
#include <QPushButton>
#include <QByteArray>
#include <QStringList>

// =====================================================
// Motor send-angle state
// After the host sends ANGLE, it must wait for ESP32 MOTOR_DONE before sending the next angle
// Angle send mode combo box:
//   0 = Single angle
//   1 = Back-and-forth: +A, -A, +A, -A ... infinite loop
//
// In back-and-forth mode:
//   - The button remains clickable and shows "Stop Back-and-Forth"
//   - Clicking stop does not interrupt the current motor move
//   - After the current MOTOR_DONE is received, the next ANGLE will not be sent
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
int gBackAndForthSentCount = 0;   // Number of back-and-forth move segments already sent

QComboBox *gAngleModeCombo = nullptr;
QLabel *gAngleModeLabel = nullptr;
QPushButton *gResetAngleButton = nullptr;

// Save Parameter 状态机：不要一次性把 PID/SPEED/CFG_SAVE 全部塞进 TCP。
// 每收到 ESP32 对上一条命令的 OK 回复后，再发送下一条命令。
bool gSaveParameterRunning = false;
QStringList gSaveParameterCommands;
int gSaveParameterStep = 0;
int gSaveParameterSequenceId = 0;

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

const char *RESET_ANGLE_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #9b59b6;"
    " color: white;"
    " font-weight: bold;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:hover {"
    " background-color: #8e44ad;"
    "}"
    "QPushButton:disabled {"
    " background-color: #bdc3c7;"
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
    // ESP32 accepts plain numbers. Keep 3 decimals and trim trailing zeros for cleaner logs.
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
    // 1st move +A, 2nd move -A, 3rd move +A ... alternating indefinitely.
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

    if (gResetAngleButton != nullptr) {
        gResetAngleButton->setEnabled(true);
    }

    if (currentAngleSendMode() == MODE_BACK_AND_FORTH) {
        ui->sendAngleButton->setText("Start Back-and-Forth");
        ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SWING_IDLE_STYLE);
    } else {
        ui->sendAngleButton->setText("Send Angle");
        ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SINGLE_IDLE_STYLE);
    }
}

void setSendAngleBusy(Ui::MainWindow *ui)
{
    ui->angleEdit->setEnabled(false);
    setAngleModeWidgetsEnabled(false);

    if (gResetAngleButton != nullptr) {
        gResetAngleButton->setEnabled(false);
    }

    if (gAngleSequenceRunning) {
        if (gAngleSequenceStopRequested) {
            ui->sendAngleButton->setEnabled(false);
            ui->sendAngleButton->setText("Stopping, wait for current move...");
            ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SWING_STOPPING_STYLE);
        } else {
            // In back-and-forth mode, the button stays clickable to request stop; it will not interrupt the current move.
            ui->sendAngleButton->setEnabled(true);
            ui->sendAngleButton->setText(
                QString("Stop Back-and-Forth (Run #%1)").arg(gBackAndForthSentCount)
                );
            ui->sendAngleButton->setStyleSheet(SEND_ANGLE_SWING_BUSY_STYLE);
        }
    } else {
        ui->sendAngleButton->setEnabled(false);
        ui->sendAngleButton->setText("Motor Moving...");
        ui->sendAngleButton->setStyleSheet(SEND_ANGLE_BUSY_STYLE);
    }
}


void hideParameterEditor(QWidget *editor, const QStringList &labelKeywords)
{
    if (editor == nullptr) {
        return;
    }

    QWidget *parent = editor->parentWidget();
    editor->hide();
    editor->setEnabled(false);

    if (parent == nullptr) {
        return;
    }

    const QList<QLabel *> labels = parent->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        QString text = label->text();
        text.remove('&');

        for (const QString &keyword : labelKeywords) {
            if (text.contains(keyword, Qt::CaseInsensitive)) {
                label->hide();
                label->setEnabled(false);
                break;
            }
        }
    }
}

void setupAngleModeCombo(Ui::MainWindow *ui)
{
    QWidget *parent = ui->sendAngleButton->parentWidget();

    if (parent == nullptr) {
        return;
    }

    // If you later add a combo box with objectName=angleModeCombo in Qt Designer,
    // this code will reuse it and will not create a duplicate.
    gAngleModeCombo = parent->findChild<QComboBox *>("angleModeCombo");
    gAngleModeLabel = parent->findChild<QLabel *>("angleModeLabel");

    if (gAngleModeCombo == nullptr) {
        gAngleModeLabel = new QLabel("Angle Mode:", parent);
        gAngleModeLabel->setObjectName("angleModeLabel");

        gAngleModeCombo = new QComboBox(parent);
        gAngleModeCombo->setObjectName("angleModeCombo");
        gAngleModeCombo->addItem("Single Angle");
        gAngleModeCombo->addItem("Back-and-Forth: +A -A Infinite");

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
            // Fallback: show below the button when there is no layout.
            const QRect buttonRect = ui->sendAngleButton->geometry();
            gAngleModeLabel->setGeometry(buttonRect.left(), buttonRect.bottom() + 8, 70, 24);
            gAngleModeCombo->setGeometry(buttonRect.left() + 75, buttonRect.bottom() + 8, 200, 24);
            gAngleModeLabel->show();
            gAngleModeCombo->show();
        }
    }

    // If you later add a QPushButton with objectName=resetAngleButton in Qt Designer,
    // this code will reuse it and will not create a duplicate.
    // Put the reset button next to Current Angle so it is visible in this UI layout.
    QWidget *resetParent = ui->currentAngleEdit->parentWidget();
    if (resetParent == nullptr) {
        resetParent = parent;
    }

    gResetAngleButton = resetParent->findChild<QPushButton *>("resetAngleButton");

    if (gResetAngleButton == nullptr) {
        gResetAngleButton = new QPushButton("Reset Angle Zero", resetParent);
        gResetAngleButton->setObjectName("resetAngleButton");
        gResetAngleButton->setStyleSheet(RESET_ANGLE_BUTTON_STYLE);
        gResetAngleButton->setToolTip("Set the ESP32 encoder angle to zero. Disabled while the motor is moving.");

        QLayout *resetLayout = resetParent->layout();

        if (QBoxLayout *boxLayout = qobject_cast<QBoxLayout *>(resetLayout)) {
            int angleIndex = boxLayout->indexOf(ui->currentAngleEdit);
            if (angleIndex < 0) {
                angleIndex = boxLayout->count() - 1;
            }

            boxLayout->insertWidget(angleIndex + 1, gResetAngleButton);
        } else if (QGridLayout *gridLayout = qobject_cast<QGridLayout *>(resetLayout)) {
            int row = 0;
            int column = 0;
            int rowSpan = 1;
            int columnSpan = 1;

            int angleIndex = gridLayout->indexOf(ui->currentAngleEdit);
            if (angleIndex >= 0) {
                gridLayout->getItemPosition(angleIndex, &row, &column, &rowSpan, &columnSpan);
                gridLayout->addWidget(gResetAngleButton, row, column + columnSpan, 1, 1);
            } else {
                gridLayout->addWidget(gResetAngleButton, gridLayout->rowCount(), 0, 1, 2);
            }
        } else {
            // Fallback for this absolute-positioned UI: place it to the right of Current Angle.
            const QRect angleRect = ui->currentAngleEdit->geometry();
            const int buttonWidth = 190;
            const int buttonHeight = (angleRect.height() > 36 ? angleRect.height() : 36);
            int x = angleRect.right() + 20;
            int y = angleRect.top();

            if (resetParent->width() > 0 && x + buttonWidth > resetParent->width()) {
                x = angleRect.left();
                y = angleRect.bottom() + 8;
            }

            gResetAngleButton->setGeometry(x, y, buttonWidth, buttonHeight);
            gResetAngleButton->show();
        }
    } else {
        gResetAngleButton->setText("Reset Angle Zero");
        gResetAngleButton->setStyleSheet(RESET_ANGLE_BUTTON_STYLE);
        gResetAngleButton->setToolTip("Set the ESP32 encoder angle to zero. Disabled while the motor is moving.");
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

    // 确保 Save Parameter 按钮一定连接到保存函数。
    // Qt Designer 自动连接依赖 objectName；这里再显式连接一次，避免按钮 objectName/自动连接异常导致点击无效。
    connect(ui->btnSaveParameter, &QPushButton::clicked,
            this, &MainWindow::on_btnSaveParameter_clicked,
            Qt::UniqueConnection);

    socket = new QTcpSocket(this);

    // =====================================================
    // Default IP and port
    // =====================================================
    ui->lineEditIP->setText("192.168.4.1");
    ui->lineEditPort->setText("8080");

    ui->labelConnectStatus->setText("Disconnected");
    ui->labelConnectStatus->setStyleSheet("color:red; font-weight:bold;");

    // =====================================================
    // Default parameter display
    // PID + single Speed parameter
    // =====================================================
    ui->kpEdit->setText("5.0");
    ui->kiEdit->setText("0.015");
    ui->kdEdit->setText("0.0");

    ui->speedEdit->setText("500");

    // CPR 和 Tolerance 由 ESP32 固件固定/默认处理，上位机不再显示和保存这两个参数。
    hideParameterEditor(findChild<QWidget *>("CPREdit"), QStringList() << "CPR" << "Encoder CPR" << "Encoder");
    hideParameterEditor(findChild<QWidget *>("toleranceEdit"), QStringList() << "Tolerance" << "TOL");

    ui->currentAngleEdit->setReadOnly(true);

    // Angle send mode combo box + initial button state
    setupAngleModeCombo(ui);

    if (gResetAngleButton != nullptr) {
        connect(gResetAngleButton, &QPushButton::clicked, this, [=]() {
            if (gMotorMoving || gAngleSequenceRunning) {
                addLog("Cannot reset angle while the motor is moving. Stop or wait for MOTOR_DONE first.");
                return;
            }

            if (socket->state() != QAbstractSocket::ConnectedState) {
                addLog("Not connected");
                return;
            }

            sendCommand("ZERO");
            addLog("Reset angle zero command sent");
        });
    }
    gMotorMoving = false;
    gAngleSequenceRunning = false;
    gAngleSequenceStopRequested = false;
    gBackAndForthBaseAngle = 0.0;
    gBackAndForthSentCount = 0;
    setSendAngleIdle(ui);

    // =====================================================
    // Input validators
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


    // =====================================================
    // Periodically read current angle
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

        // Read current ESP32 config automatically after connection
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

        // Clear motor busy state on disconnect to avoid a permanently locked button
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
    QByteArray data = line.toUtf8();
    qint64 written = socket->write(data);

    if (written != data.size()) {
        addLog("Socket write warning: " + socket->errorString());
    }

    socket->flush();

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

}

void MainWindow::handleEsp32Line(const QString &line)
{
    // Save Parameter 状态机：
    // step 0: 已发 PID，等待 OK PID
    // step 1: 已发 SPEED，等待 OK SPEED
    // step 2: 已发 CFG_SAVE，等待 OK CFG_SAVE
    // step 3: 已发 CFG?，等待 CFG 回读
    if (gSaveParameterRunning) {
        if (line.startsWith("ERR") || line.startsWith("UNKNOWN CMD")) {
            gSaveParameterRunning = false;
            gSaveParameterCommands.clear();
            gSaveParameterStep = 0;
            ui->btnSaveParameter->setEnabled(true);
            addLog("Save failed, ESP32 replied: " + line);
            return;
        }

        bool advanceSaveStep = false;

        if (gSaveParameterStep == 0 && line.startsWith("OK PID")) {
            updateConfigEditsFromLine(line);
            advanceSaveStep = true;
        } else if (gSaveParameterStep == 1 && line.startsWith("OK SPEED")) {
            updateConfigEditsFromLine(line);
            advanceSaveStep = true;
        } else if (gSaveParameterStep == 2 && line.startsWith("OK CFG_SAVE")) {
            updateConfigEditsFromLine(line);
            addLog("ESP32 confirmed: PID and Speed written to Flash");
            advanceSaveStep = true;
        } else if (gSaveParameterStep == 3 && line.startsWith("CFG ")) {
            updateConfigEditsFromLine(line);
            gSaveParameterRunning = false;
            gSaveParameterCommands.clear();
            gSaveParameterStep = 0;
            ui->btnSaveParameter->setEnabled(true);
            addLog("Save verified by CFG?: " + line);
            return;
        }

        if (advanceSaveStep) {
            gSaveParameterStep++;

            if (gSaveParameterStep < gSaveParameterCommands.size()) {
                const int sequenceId = gSaveParameterSequenceId;
                const QString nextCmd = gSaveParameterCommands.at(gSaveParameterStep);

                QTimer::singleShot(80, this, [=]() {
                    if (!gSaveParameterRunning || sequenceId != gSaveParameterSequenceId) {
                        return;
                    }

                    if (socket->state() != QAbstractSocket::ConnectedState) {
                        gSaveParameterRunning = false;
                        gSaveParameterCommands.clear();
                        gSaveParameterStep = 0;
                        ui->btnSaveParameter->setEnabled(true);
                        addLog("Save stopped: disconnected before sending " + nextCmd);
                        return;
                    }

                    sendCommand(nextCmd, true);
                });
            }

            return;
        }
        // 其他行，比如周期 ANGLE? 回包，继续走普通解析，不影响保存状态机。
    }

    // ESP32 sends this only after the motor actually stops: MOTOR_DONE STATUS=OK ...
    // Single mode: unlock the button after MOTOR_DONE.
    // Back-and-forth mode: after MOTOR_DONE, send the next segment unless stop was requested;
    // if stop was already requested, finish after this MOTOR_DONE and do not send the next ANGLE.
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

            // Delay 100 ms before sending the next segment so ESP32 MOTOR_DONE and host logs are processed first.
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

    // If ESP32 rejects the ANGLE command, there will be no MOTOR_DONE, so unlock here.
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

    // ZERO / HOME_ZERO reply: OK HOME_ZERO ... CURRENT_ANGLE=0.00
    if (line.startsWith("OK HOME_ZERO")) {
        QString angle = valueFromLine(line, "CURRENT_ANGLE");

        if (angle.isEmpty()) {
            angle = "0";
        }

        ui->currentAngleEdit->setText(angle);
        addLog("ESP32 angle reset to zero");
        return;
    }

    // ESP32 confirmed that RAM parameters have been saved to Flash.
    if (line.startsWith("OK CFG_SAVE")) {
        updateConfigEditsFromLine(line);
        addLog("ESP32 confirmed: PID and Speed saved to Flash");
        return;
    }

    // CFG KP=... KI=... KD=... SPEED=...
    if (line.startsWith("CFG ") ||
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
    // When back-and-forth is running, the button requests stop.
    // It does not interrupt the current motor move; it stops sending after the current MOTOR_DONE.
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
        // Back-and-forth mode: +A, -A, +A, -A ... infinite loop.
        // Each segment waits for ESP32 MOTOR_DONE before the next segment is sent.
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
        // Single angle mode.
        gAngleSequenceRunning = false;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = 0.0;
        gBackAndForthSentCount = 0;
    }

    // Lock state first: single mode unlocks after MOTOR_DONE;
    // back-and-forth mode continues after each MOTOR_DONE until the stop button is pressed.
    gMotorMoving = true;
    setSendAngleBusy(ui);

    QString cmd = QString("ANGLE:%1").arg(formatAngleForCommand(firstAngle));
    sendCommand(cmd);
}

// =====================================================
// Save Parameter
// One button: send PID + Speed to ESP32 RAM, then save to Flash
// Commands:
// PID:Kp,Ki,Kd
// SPEED:Speed
// CFG_SAVE
// CFG?
// =====================================================
void MainWindow::on_btnSaveParameter_clicked()
{
    if (gSaveParameterRunning) {
        addLog("Save already running, please wait for CFG? verification");
        return;
    }

    if (gMotorMoving || gAngleSequenceRunning) {
        addLog("Motor is moving, please save parameters after MOTOR_DONE");
        return;
    }

    if (socket->state() != QAbstractSocket::ConnectedState) {
        addLog("Not connected");
        return;
    }

    QString kp = ui->kpEdit->text().trimmed();
    QString ki = ui->kiEdit->text().trimmed();
    QString kd = ui->kdEdit->text().trimmed();
    QString speed = ui->speedEdit->text().trimmed();

    if (kp.isEmpty() || ki.isEmpty() || kd.isEmpty() || speed.isEmpty()) {
        addLog("Parameter input empty");
        return;
    }

    bool okKp = false;
    bool okKi = false;
    bool okKd = false;
    bool okSpeed = false;

    double kpVal = kp.toDouble(&okKp);
    double kiVal = ki.toDouble(&okKi);
    double kdVal = kd.toDouble(&okKd);
    double speedVal = speed.toDouble(&okSpeed);

    if (!okKp || !okKi || !okKd || !okSpeed) {
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

    gSaveParameterCommands.clear();
    gSaveParameterCommands << QString("PID:%1,%2,%3").arg(kp).arg(ki).arg(kd)
                           << QString("SPEED:%1").arg(speed)
                           << QString("CFG_SAVE")
                           << QString("CFG?");

    gSaveParameterRunning = true;
    gSaveParameterStep = 0;
    gSaveParameterSequenceId++;
    ui->btnSaveParameter->setEnabled(false);

    addLog("Save started: PID -> SPEED -> CFG_SAVE -> CFG?");
    sendCommand(gSaveParameterCommands.at(0), true);

    const int sequenceId = gSaveParameterSequenceId;
    QTimer::singleShot(6000, this, [=]() {
        if (gSaveParameterRunning && sequenceId == gSaveParameterSequenceId) {
            gSaveParameterRunning = false;
            gSaveParameterCommands.clear();
            gSaveParameterStep = 0;
            ui->btnSaveParameter->setEnabled(true);
            addLog("Save timeout: did not receive expected ESP32 reply. Check ESP32 log for OK PID / OK SPEED / OK CFG_SAVE / CFG");
        }
    });
}

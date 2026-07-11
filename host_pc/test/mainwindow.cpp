#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QFileInfo>
#include <QUrl>
#include <QDesktopServices>
#include <QDebug>
#include <QDateTime>
#include <QTimer>
#include <QRegularExpression>
#include <QDoubleValidator>
#include <QComboBox>
#include <QAbstractItemView>
#include <QLabel>
#include <QLayout>
#include <QBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QFormLayout>
#include <QByteArray>
#include <QStringList>
#include <QEvent>
#include <QVariant>
#include <QFont>
#include <QSize>
#include <QRect>
#include <QtGlobal>
#include <QFrame>
#include <QPixmap>
#include <QSlider>
#include <functional>

// =====================================================
// Motor send-angle state
// After the host sends ANGLE, it must wait for ESP32 MOTOR_DONE before sending the next angle
// Angle send mode combo box:
//   0 = Single angle
//   1 = Back-and-forth around the start angle:
//       first move +A, then -2A, +2A, -2A ... infinite loop
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
QLabel *gSendAngleStatusLabel = nullptr;
QPushButton *gEstopButton = nullptr;
QPushButton *gRestartEsp32Button = nullptr;
QPushButton *gOpenUserFileButton = nullptr;
QLineEdit *gAbsAngleLimitEdit = nullptr;
QLabel *gAbsAngleLimitLabel = nullptr;
QLabel *gCommTextLabel = nullptr; // 注释 / Button Help
QLabel *gSpeedUnitLabel = nullptr;  // Speed unit: Step/s
QFrame *gConnectionFrame = nullptr; //边框
QFrame *gWifiFrame = nullptr;
QLabel *gWifiIconLabel = nullptr;
QLabel *gWifiTitleLabel = nullptr;
QFrame *gStatusLogFrame = nullptr;  // Status Log 边框
QFrame *gLedControlFrame = nullptr; // LED Control 边框
QFrame *gParameterFrame = nullptr;  // PID / Speed / Angle Limit 参数边框
QFrame *gAngleControlFrame = nullptr; // Motor Angle / Send Angle 区域边框
QLabel *gMiddleImageLabel = nullptr; // 参数模块和 LED 模块中间的图片接口

QPushButton *gLedToggleButton = nullptr;
QSlider *gLedSliderR = nullptr;
QSlider *gLedSliderG = nullptr;
QSlider *gLedSliderB = nullptr;
QLabel *gLedColorPreview = nullptr;
QTimer *gEstopLedFlashTimer = nullptr;
bool gEstopLedFlashRedPhase = true;
std::function<void()> gEstopLedFlashStep;
std::function<void()> gEstopLedFlashStop;

// =====================================================
// Whole-window linear scaling state
// =====================================================
QSize gBaseCentralSize;
bool gScaleReady = false;
bool gScaleApplying = false;

bool gBaseGeometrySaved(QWidget *w)
{
    return w != nullptr && w->property("baseGeometry").isValid();
}


bool gEstopActive = false;




// sendCommand 会在这里记录最近一次 TCP 写入是否成功，供 Send Angle 状态显示使用。
bool gLastCommandWriteOk = false;

// Save Parameter 状态机：不要一次性把 PID/SPEED/ANGLE_LIMIT/CFG_SAVE 全部塞进 TCP。
// 每收到 ESP32 对上一条命令的 OK 回复后，再发送下一条命令。
bool gSaveParameterRunning = false;
QStringList gSaveParameterCommands;
int gSaveParameterStep = 0;
int gSaveParameterSequenceId = 0;

const char *SEND_ANGLE_SINGLE_IDLE_STYLE =
    "QPushButton {"
    " background-color: #3498db;"
    " color: white;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:hover {"
    " background-color: #2980b9;"
    "}"
    "QPushButton:pressed {"
    " background-color: #21618c;"
    "}";

const char *SEND_ANGLE_SWING_IDLE_STYLE =
    "QPushButton {"
    " background-color: #3498db;"
    " color: white;"
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
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:disabled {"
    " background-color: #f39c12;"
    " color: white;"
    "}";

const char *SEND_ANGLE_SWING_BUSY_STYLE =
    "QPushButton {"
    " background-color: #e74c3c;"
    " color: white;"
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
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:disabled {"
    " background-color: #7f8c8d;"
    " color: white;"
    "}";

const char *GREEN_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #3498db;"
    " color: white;"
    " border-radius: 6px;"
    " padding: 6px 12px;"
    "}"
    "QPushButton:hover {"
    " background-color: #2980b9;"
    "}"
    "QPushButton:pressed {"
    " background-color: #21618c;"
    "}"
    "QPushButton:disabled {"
    " background-color: #aed6f1;"
    " color: white;"
    "}";

const char *RESET_ANGLE_BUTTON_STYLE = GREEN_BUTTON_STYLE;

const char *ESTOP_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #e53935;"
    " color: white;"
    " border-radius: 6px;"
    " padding: 8px 16px;"
    "}"
    "QPushButton:hover {"
    " background-color: #c62828;"
    "}"
    "QPushButton:pressed {"
    " background-color: #b71c1c;"
    "}";

const char *USER_FILE_BUTTON_STYLE = GREEN_BUTTON_STYLE;

const char *ESTOP_UNLOCK_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #3498db;"
    " color: white;"
    " border-radius: 6px;"
    " padding: 8px 16px;"
    "}"
    "QPushButton:hover {"
    " background-color: #2980b9;"
    "}"
    "QPushButton:pressed {"
    " background-color: #21618c;"
    "}";

const char *RESTART_ESP32_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #fb8c00;"
    " color: white;"
    " border-radius: 6px;"
    " padding: 8px 14px;"
    "}"
    "QPushButton:hover {"
    " background-color: #ef6c00;"
    "}"
    "QPushButton:pressed {"
    " background-color: #e65100;"
    "}";

const char *SEND_ANGLE_STATUS_IDLE_STYLE =
    "QLabel {"
    " color: #7f8c8d;"
    "}";

const char *SEND_ANGLE_STATUS_SUCCESS_STYLE =
    "QLabel {"
    " color: #1e88e5;"
    "}";

const char *SEND_ANGLE_STATUS_FAILED_STYLE =
    "QLabel {"
    " color: #e74c3c;"
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
    // The first segment is the user input A.
    // After that, move between start + A and start - A:
    // sent count 1 -> next -2A, sent count 2 -> next +2A, sent count 3 -> next -2A ...
    if ((gBackAndForthSentCount % 2) == 1) {
        return -2.0 * gBackAndForthBaseAngle;
    }

    return 2.0 * gBackAndForthBaseAngle;
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

void setSendAngleStatusIdle(const QString &text = QString())
{
    if (gSendAngleStatusLabel == nullptr) {
        return;
    }

    gSendAngleStatusLabel->setText(text);
    gSendAngleStatusLabel->setStyleSheet(SEND_ANGLE_STATUS_IDLE_STYLE);
}

void setSendAngleStatusSuccess()
{
    if (gSendAngleStatusLabel == nullptr) {
        return;
    }

    gSendAngleStatusLabel->setText("Success");
    gSendAngleStatusLabel->setStyleSheet(SEND_ANGLE_STATUS_SUCCESS_STYLE);
}

void setSendAngleStatusFailed()
{
    if (gSendAngleStatusLabel == nullptr) {
        return;
    }

    gSendAngleStatusLabel->setText("Failed");
    gSendAngleStatusLabel->setStyleSheet(SEND_ANGLE_STATUS_FAILED_STYLE);
}

void setupSendAngleStatusLabel(Ui::MainWindow *ui, QWidget *parent)
{
    if (ui == nullptr || ui->textBrowserLog == nullptr) {
        return;
    }

    // 放到 Status Log 下方那个日志框(textBrowserLog)的右边，
    // 不再放在 Send Angle 按钮左边。
    QWidget *statusParent = ui->textBrowserLog->parentWidget();

    if (statusParent == nullptr) {
        statusParent = parent;
    }

    if (statusParent == nullptr) {
        return;
    }

    gSendAngleStatusLabel = statusParent->findChild<QLabel *>("sendAngleStatusLabel");

    if (gSendAngleStatusLabel == nullptr) {
        gSendAngleStatusLabel = new QLabel("", statusParent);
        gSendAngleStatusLabel->setObjectName("sendAngleStatusLabel");
    } else if (gSendAngleStatusLabel->parentWidget() != statusParent) {
        gSendAngleStatusLabel->setParent(statusParent);
    }

    gSendAngleStatusLabel->setAlignment(Qt::AlignCenter);
    gSendAngleStatusLabel->setMinimumSize(170, 48);
    gSendAngleStatusLabel->setStyleSheet(SEND_ANGLE_STATUS_IDLE_STYLE);

    const QRect logRect = ui->textBrowserLog->geometry();
    const int labelWidth = 190;
    const int labelHeight = 58;
    const int gap = 18;

    int x = logRect.right() + gap;
    int y = logRect.top() + (logRect.height() - labelHeight) / 2;

    // 如果窗口太窄导致右边放不下，就退到日志框下方，保证能看见。
    if (statusParent->width() > 0 && x + labelWidth > statusParent->width()) {
        x = logRect.left();
        y = logRect.bottom() + 10;
    }

    if (y < 0) {
        y = logRect.top();
    }

    gSendAngleStatusLabel->setGeometry(x, y, labelWidth, labelHeight);
    gSendAngleStatusLabel->show();
    gSendAngleStatusLabel->raise();
}

void setLedControlWidgetsEnabled(bool enabled)
{
    if (gLedToggleButton != nullptr) {
        gLedToggleButton->setEnabled(enabled);
        gLedToggleButton->setToolTip(enabled
                                         ? "Toggle RGB LED."
                                         : "LED control is disabled while ESTOP is active.");
    }

    if (gLedSliderR != nullptr) {
        gLedSliderR->setEnabled(enabled);
    }

    if (gLedSliderG != nullptr) {
        gLedSliderG->setEnabled(enabled);
    }

    if (gLedSliderB != nullptr) {
        gLedSliderB->setEnabled(enabled);
    }
}

void setEstopButtonState(bool active)
{
    const bool wasActive = gEstopActive;
    gEstopActive = active;

    if (active) {
        setLedControlWidgetsEnabled(false);

        if (!wasActive) {
            gEstopLedFlashRedPhase = true;

            if (gEstopLedFlashStep) {
                gEstopLedFlashStep();
            }

            if (gEstopLedFlashTimer != nullptr) {
                gEstopLedFlashTimer->start();
            }
        }
    } else {
        if (wasActive && gEstopLedFlashStop) {
            gEstopLedFlashStop();
        }

        setLedControlWidgetsEnabled(true);
    }

    if (gEstopButton == nullptr) {
        return;
    }

    if (active) {
        gEstopButton->setText("UNLOCK\nESTOP");
        gEstopButton->setStyleSheet(ESTOP_UNLOCK_BUTTON_STYLE);
        gEstopButton->setToolTip("Clear emergency stop: send ESTOP_CLEAR to ESP32 and re-enable the A4988 driver.");
    } else {
        gEstopButton->setText("ESTOP");
        gEstopButton->setStyleSheet(ESTOP_BUTTON_STYLE);
        gEstopButton->setToolTip("Emergency stop: send ESTOP to ESP32 and disable the A4988 driver immediately.");
    }
}

void setupAbsAngleLimitEditor(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->speedEdit == nullptr) {
        return;
    }

    QWidget *parent = ui->speedEdit->parentWidget();

    if (parent == nullptr) {
        return;
    }

    gAbsAngleLimitEdit = parent->findChild<QLineEdit *>("absAngleLimitEdit");
    gAbsAngleLimitLabel = parent->findChild<QLabel *>("absAngleLimitLabel");

    if (gAbsAngleLimitLabel == nullptr) {
        gAbsAngleLimitLabel = new QLabel("Angle Limit:", parent);
        gAbsAngleLimitLabel->setObjectName("absAngleLimitLabel");
        gAbsAngleLimitLabel->setToolTip("Maximum allowed absolute Current Angle. 0 disables the angle limit.");
    }

    if (gAbsAngleLimitEdit == nullptr) {
        gAbsAngleLimitEdit = new QLineEdit(parent);
        gAbsAngleLimitEdit->setObjectName("absAngleLimitEdit");
        gAbsAngleLimitEdit->setText("60");
        gAbsAngleLimitEdit->setPlaceholderText("0 = OFF, e.g. 60");
        gAbsAngleLimitEdit->setToolTip("Send ANGLE_LIMIT:<value>. 0 disables the absolute angle limit.");

        QLayout *layout = parent->layout();

        if (QFormLayout *formLayout = qobject_cast<QFormLayout *>(layout)) {
            int row = -1;
            QFormLayout::ItemRole role;
            formLayout->getWidgetPosition(ui->speedEdit, &row, &role);

            if (row >= 0) {
                formLayout->insertRow(row + 1, gAbsAngleLimitLabel, gAbsAngleLimitEdit);
            } else {
                formLayout->addRow(gAbsAngleLimitLabel, gAbsAngleLimitEdit);
            }
        } else if (QBoxLayout *boxLayout = qobject_cast<QBoxLayout *>(layout)) {
            int speedIndex = boxLayout->indexOf(ui->speedEdit);
            if (speedIndex < 0) {
                speedIndex = boxLayout->count() - 1;
            }

            boxLayout->insertWidget(speedIndex + 1, gAbsAngleLimitLabel);
            boxLayout->insertWidget(speedIndex + 2, gAbsAngleLimitEdit);
        } else if (QGridLayout *gridLayout = qobject_cast<QGridLayout *>(layout)) {
            int row = 0;
            int column = 0;
            int rowSpan = 1;
            int columnSpan = 1;

            int speedIndex = gridLayout->indexOf(ui->speedEdit);
            if (speedIndex >= 0) {
                gridLayout->getItemPosition(speedIndex, &row, &column, &rowSpan, &columnSpan);
                gridLayout->addWidget(gAbsAngleLimitLabel, row + rowSpan, column > 0 ? column - 1 : column, 1, 1);
                gridLayout->addWidget(gAbsAngleLimitEdit, row + rowSpan, column, 1, columnSpan);
            } else {
                gridLayout->addWidget(gAbsAngleLimitLabel, gridLayout->rowCount(), 0, 1, 1);
                gridLayout->addWidget(gAbsAngleLimitEdit, gridLayout->rowCount() - 1, 1, 1, 1);
            }
        } else {
            // Fallback for absolute-positioned UI: place it directly below Speed.
            const QRect speedRect = ui->speedEdit->geometry();
            const int gap = 8;
            const int labelWidth = 130;
            const int editWidth = speedRect.width() > 80 ? speedRect.width() : 100;
            const int h = speedRect.height() > 24 ? speedRect.height() : 28;
            int labelX = speedRect.left() - labelWidth - 10;
            int editX = speedRect.left();
            int y = speedRect.bottom() + gap;

            if (labelX < 0) {
                labelX = speedRect.left();
                editX = speedRect.left() + labelWidth + 10;
            }

            gAbsAngleLimitLabel->setGeometry(labelX, y, labelWidth, h);
            gAbsAngleLimitEdit->setGeometry(editX, y, editWidth, h);
            gAbsAngleLimitLabel->show();
            gAbsAngleLimitEdit->show();
        }
    } else {
        if (gAbsAngleLimitEdit->text().trimmed().isEmpty()) {
            gAbsAngleLimitEdit->setText("60");
        }
        gAbsAngleLimitEdit->setPlaceholderText("0 = OFF, e.g. 60");
        gAbsAngleLimitEdit->setToolTip("Send ANGLE_LIMIT:<value>. 0 disables the absolute angle limit.");
    }
}

void setupEstopButton(Ui::MainWindow *ui, QWidget *parent)
{
    if (parent == nullptr || ui == nullptr || ui->sendAngleButton == nullptr) {
        return;
    }

    gEstopButton = parent->findChild<QPushButton *>("estopButton");

    if (gEstopButton == nullptr) {
        gEstopButton = new QPushButton("ESTOP", parent);
        gEstopButton->setObjectName("estopButton");
        gEstopButton->setMinimumHeight(48);
        gEstopButton->setStyleSheet(ESTOP_BUTTON_STYLE);
        gEstopButton->setToolTip("Emergency stop: send ESTOP to ESP32 and disable the A4988 driver immediately.");

        QLayout *layout = parent->layout();

        if (QBoxLayout *boxLayout = qobject_cast<QBoxLayout *>(layout)) {
            int buttonIndex = boxLayout->indexOf(ui->sendAngleButton);
            if (buttonIndex < 0) {
                buttonIndex = boxLayout->count();
            }

            // 放在 Send Angle 按钮上方；如果布局是横向，仍尽量插到按钮前面。
            boxLayout->insertWidget(buttonIndex, gEstopButton);
        } else if (QGridLayout *gridLayout = qobject_cast<QGridLayout *>(layout)) {
            int row = 0;
            int column = 0;
            int rowSpan = 1;
            int columnSpan = 1;

            int buttonIndex = gridLayout->indexOf(ui->sendAngleButton);
            if (buttonIndex >= 0) {
                gridLayout->getItemPosition(buttonIndex, &row, &column, &rowSpan, &columnSpan);

                if (row > 0) {
                    gridLayout->addWidget(gEstopButton, row - 1, column, 1, columnSpan);
                } else {
                    gridLayout->addWidget(gEstopButton, row + rowSpan, column, 1, columnSpan);
                }
            } else {
                gridLayout->addWidget(gEstopButton, gridLayout->rowCount(), 0, 1, 2);
            }
        } else {
            // Fallback for this absolute-positioned UI: place it directly above Send Angle.
            const QRect buttonRect = ui->sendAngleButton->geometry();
            const int estopHeight = 58;
            int x = buttonRect.left();
            int y = buttonRect.top() - estopHeight - 14;

            if (y < 0) {
                y = buttonRect.bottom() + 14;
            }

            gEstopButton->setGeometry(x, y, buttonRect.width(), estopHeight);
            gEstopButton->show();
        }
    } else {
        gEstopButton->setMinimumHeight(48);
    }

    setEstopButtonState(gEstopActive);
}

void positionEstopRestartTopRight(Ui::MainWindow *ui)
{
    if (ui == nullptr ||
        ui->centralwidget == nullptr ||
        gEstopButton == nullptr ||
        gRestartEsp32Button == nullptr ||
        gOpenUserFileButton == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    // 右上角横向排列：User TXT 在最左，Restart ESP32 在中间，ESTOP 保持原来的右上角位置。
    const int buttonSize = 120;
    const int gap = 18;
    const int userGroupGap = 100; // User TXT 再往左一些，和 Restart / ESTOP 这一组拉开距离。
    const int marginRight = 35;
    const int marginTop = 25;

    const QRect wifiRect = (gConnectionFrame != nullptr)
                               ? gConnectionFrame->geometry()
                               : QRect(20, 40, 800, 140);

    // ESTOP 的位置保持不变：仍然贴近右上角。
    int estopX = parent->width() - marginRight - buttonSize;

    // 如果窗口宽度还没有初始化完成，至少放到 Wi-Fi 框右边，避免盖住 Wi-Fi 区域。
    const int minEstopX = wifiRect.right() + 35;
    if (parent->width() <= minEstopX + buttonSize || estopX < minEstopX) {
        estopX = minEstopX;
    }

    int topY = marginTop;
    if (topY < 20) {
        topY = 20;
    }

    const int restartX = estopX - gap - buttonSize;
    const int userX = restartX - userGroupGap - buttonSize;

    gOpenUserFileButton->setGeometry(userX, topY, buttonSize, buttonSize);
    gRestartEsp32Button->setGeometry(restartX, topY, buttonSize, buttonSize);
    gEstopButton->setGeometry(estopX, topY, buttonSize, buttonSize);

    gOpenUserFileButton->setMinimumSize(buttonSize, buttonSize);
    gOpenUserFileButton->setMaximumSize(buttonSize, buttonSize);
    gEstopButton->setMinimumSize(buttonSize, buttonSize);
    gEstopButton->setMaximumSize(buttonSize, buttonSize);
    gRestartEsp32Button->setMinimumSize(buttonSize, buttonSize);
    gRestartEsp32Button->setMaximumSize(buttonSize, buttonSize);

    // 换行后在正方形按钮里更居中、更不容易被截断。
    gOpenUserFileButton->setText("User\nTXT");
    gRestartEsp32Button->setText("Restart\nESP32");

    gEstopButton->show();
    gRestartEsp32Button->show();
    gOpenUserFileButton->show();

    gEstopButton->raise();
    gRestartEsp32Button->raise();
    gOpenUserFileButton->raise();
}

void setupRestartEsp32Button(Ui::MainWindow *ui, QWidget *parent)
{
    if (parent == nullptr || ui == nullptr || gEstopButton == nullptr) {
        return;
    }

    gRestartEsp32Button = parent->findChild<QPushButton *>("restartEsp32Button");

    if (gRestartEsp32Button == nullptr) {
        gRestartEsp32Button = new QPushButton("Restart ESP32", parent);
        gRestartEsp32Button->setObjectName("restartEsp32Button");
        gRestartEsp32Button->setMinimumHeight(48);
        gRestartEsp32Button->setStyleSheet(RESTART_ESP32_BUTTON_STYLE);
        gRestartEsp32Button->setToolTip("Restart ESP32 by sending ESTOP_REBOOT. The ESP32 will reboot with ESTOP latched until UNLOCK ESTOP is pressed.");

        QLayout *layout = parent->layout();

        if (QBoxLayout *boxLayout = qobject_cast<QBoxLayout *>(layout)) {
            int estopIndex = boxLayout->indexOf(gEstopButton);
            if (estopIndex < 0) {
                estopIndex = boxLayout->count() - 1;
            }
            boxLayout->insertWidget(estopIndex + 1, gRestartEsp32Button);
        } else if (QGridLayout *gridLayout = qobject_cast<QGridLayout *>(layout)) {
            int row = 0;
            int column = 0;
            int rowSpan = 1;
            int columnSpan = 1;

            int estopIndex = gridLayout->indexOf(gEstopButton);
            if (estopIndex >= 0) {
                gridLayout->getItemPosition(estopIndex, &row, &column, &rowSpan, &columnSpan);
                gridLayout->addWidget(gRestartEsp32Button, row, column + columnSpan, rowSpan, 1);
            } else {
                gridLayout->addWidget(gRestartEsp32Button, gridLayout->rowCount(), 2, 1, 1);
            }
        } else {
            // Fallback for this absolute-positioned UI: place it on the right side of ESTOP.
            const QRect estopRect = gEstopButton->geometry();
            const int buttonWidth = 250;
            const int buttonHeight = estopRect.height() > 0 ? estopRect.height() : 58;
            const int gap = 100;

            int x = estopRect.right() + gap;
            int y = estopRect.top();

            // 如果当前窗口右侧空间不够，就仍然尽量靠右显示，不覆盖 ESTOP。
            // if (parent->width() > 0 && x + buttonWidth > parent->width()) {
            //     x = estopRect.left();
            //     y = estopRect.bottom() + 10;
            // }

            gRestartEsp32Button->setGeometry(x, y, buttonWidth, buttonHeight);
            gRestartEsp32Button->show();
        }
    } else {
        gRestartEsp32Button->setText("Restart ESP32");
        gRestartEsp32Button->setMinimumHeight(48);
        gRestartEsp32Button->setStyleSheet(RESTART_ESP32_BUTTON_STYLE);
        gRestartEsp32Button->setToolTip("Restart ESP32 by sending ESTOP_REBOOT. The ESP32 will reboot with ESTOP latched until UNLOCK ESTOP is pressed.");
    }
}


void setupOpenUserFileButton(Ui::MainWindow *ui, QWidget *parent)
{
    if (parent == nullptr || ui == nullptr || gEstopButton == nullptr) {
        return;
    }

    gOpenUserFileButton = parent->findChild<QPushButton *>("openUserFileButton");

    if (gOpenUserFileButton == nullptr) {
        gOpenUserFileButton = new QPushButton("User\nTXT", parent);
        gOpenUserFileButton->setObjectName("openUserFileButton");
        gOpenUserFileButton->setMinimumHeight(48);
        gOpenUserFileButton->setStyleSheet(USER_FILE_BUTTON_STYLE);
        gOpenUserFileButton->setToolTip("Open local user.txt. Put user.txt in the same folder as the executable.");

        QLayout *layout = parent->layout();

        if (QBoxLayout *boxLayout = qobject_cast<QBoxLayout *>(layout)) {
            int estopIndex = boxLayout->indexOf(gEstopButton);
            if (estopIndex < 0) {
                estopIndex = boxLayout->count();
            }
            boxLayout->insertWidget(estopIndex, gOpenUserFileButton);
        } else if (QGridLayout *gridLayout = qobject_cast<QGridLayout *>(layout)) {
            int row = 0;
            int column = 0;
            int rowSpan = 1;
            int columnSpan = 1;

            int estopIndex = gridLayout->indexOf(gEstopButton);
            if (estopIndex >= 0) {
                gridLayout->getItemPosition(estopIndex, &row, &column, &rowSpan, &columnSpan);
                if (row > 0) {
                    gridLayout->addWidget(gOpenUserFileButton, row - 1, column, 1, columnSpan);
                } else {
                    gridLayout->addWidget(gOpenUserFileButton, row + rowSpan, column, 1, columnSpan);
                }
            } else {
                gridLayout->addWidget(gOpenUserFileButton, gridLayout->rowCount(), 0, 1, 2);
            }
        } else {
            // Fallback for absolute-positioned UI. The final position is set by positionEstopRestartTopRight().
            const QRect estopRect = gEstopButton->geometry();
            const int buttonHeight = estopRect.height() > 0 ? estopRect.height() : 58;
            int x = estopRect.left();
            int y = estopRect.top() - buttonHeight - 14;
            if (y < 0) {
                y = estopRect.bottom() + 14;
            }
            gOpenUserFileButton->setGeometry(x, y, estopRect.width(), buttonHeight);
            gOpenUserFileButton->show();
        }
    } else {
        gOpenUserFileButton->setText("User\nTXT");
        gOpenUserFileButton->setMinimumHeight(48);
        gOpenUserFileButton->setStyleSheet(USER_FILE_BUTTON_STYLE);
        gOpenUserFileButton->setToolTip("Open local user.txt. Put user.txt in the same folder as the executable.");
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

    setupSendAngleStatusLabel(ui, parent);
    setupEstopButton(ui, parent);
    setupRestartEsp32Button(ui, parent);
    setupOpenUserFileButton(ui, parent);

    // If you later add a combo box with objectName=angleModeCombo in Qt Designer,
    // this code will reuse it and will not create a duplicate.
    gAngleModeCombo = parent->findChild<QComboBox *>("angleModeCombo");
    gAngleModeLabel = parent->findChild<QLabel *>("angleModeLabel");

    if (gAngleModeCombo == nullptr) {
        gAngleModeLabel = new QLabel("Mode:", parent);
        gAngleModeLabel->setObjectName("angleModeLabel");

        gAngleModeCombo = new QComboBox(parent);
        gAngleModeCombo->setObjectName("angleModeCombo");
        gAngleModeCombo->addItem("Single Angle");
        gAngleModeCombo->addItem("Back-and-Forth");

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
            const int buttonWidth = 250;
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

    // 下拉框和弹窗保持适中宽度：比之前小，但仍能看清 Back-and-Forth。
    if (gAngleModeCombo != nullptr) {
        gAngleModeCombo->setMinimumContentsLength(24);
        gAngleModeCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);

        if (gAngleModeCombo->view() != nullptr) {
            gAngleModeCombo->view()->setMinimumWidth(340);
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

void setupCommText(QWidget *parent)
{
    if (parent == nullptr) {
        return;
    }

    gCommTextLabel = parent->findChild<QLabel *>("CommTextLabel");

    if (gCommTextLabel == nullptr) {
        gCommTextLabel = new QLabel(parent);
        gCommTextLabel->setObjectName("CommTextLabel");
    }

    gCommTextLabel->setText(
        "Button Help\n"
        "\n"
        "Connect: connect / disconnect ESP32\n"
        "LED ON/OFF: toggle RGB LED\n"
        "ESTOP: emergency stop motor\n"
        "Restart ESP32: reboot ESP32\n"
        "Send Angle: send motor angle command\n"
        "Save Parameter: save KP/KI/KD, Speed and Limit\n"
        "Reset Angle Zero: set current angle to 0\n"
        "Angle Mode: single move or back-and-forth"
        );

    gCommTextLabel->setWordWrap(true);
    gCommTextLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);

    gCommTextLabel->setStyleSheet(
        "QLabel {"
        " background-color: #f8f9fa;"
        " color: #2c3e50;"
        " border: 2px solid #34495e;"
        " border-radius: 8px;"
        " padding: 10px;"
        "}"
        );

    const int boxWidth = 330;
    const int boxHeight = 230;
    const int marginRight = 35;
    const int marginTop = 75;

    int x = parent->width() - boxWidth - marginRight;
    int y = marginTop;

    if (x < 0) {
        x = 0;
    }

    gCommTextLabel->setGeometry(x, y, boxWidth, boxHeight);
    gCommTextLabel->show();
    gCommTextLabel->raise();
}


void positionCommTextTopRight(QWidget *parent)
{
    if (parent == nullptr || gCommTextLabel == nullptr) {
        return;
    }

    const int boxWidth = 360;
    const int boxHeight = 260;
    const int marginRight = 35;
    const int marginTop = 70;

    int x = parent->width() - boxWidth - marginRight;
    int y = marginTop;

    if (x < 0) {
        x = 0;
    }

    gCommTextLabel->setGeometry(x, y, boxWidth, boxHeight);
    gCommTextLabel->show();
    gCommTextLabel->raise();
}

void setupSpeedUnitLabel(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->speedEdit == nullptr) {
        return;
    }

    QWidget *parent = ui->speedEdit->parentWidget();

    if (parent == nullptr) {
        return;
    }

    gSpeedUnitLabel = parent->findChild<QLabel *>("speedUnitLabel");

    if (gSpeedUnitLabel == nullptr) {
        gSpeedUnitLabel = new QLabel("Step/s", parent);
        gSpeedUnitLabel->setObjectName("speedUnitLabel");
    }

    gSpeedUnitLabel->setText("Step/s");
    gSpeedUnitLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    gSpeedUnitLabel->setStyleSheet(
        "QLabel {"
        " color: #2c3e50;"
        "}"
        );

    const QRect speedRect = ui->speedEdit->geometry();
    const int x = speedRect.right() + 12;
    const int y = speedRect.top();
    const int w = 90;
    const int h = speedRect.height() > 0 ? speedRect.height() : 28;

    gSpeedUnitLabel->setGeometry(x, y, w, h);
    gSpeedUnitLabel->show();
    gSpeedUnitLabel->raise();
}


void removeFontRulesFromStyleSheet(QWidget *w);

void applyModeFontToAllWidgets(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    QFont modeFont = (gAngleModeLabel != nullptr)
                         ? gAngleModeLabel->font()
                         : QApplication::font();
    modeFont.setBold(false);
    modeFont.setItalic(false);

    ui->centralwidget->setFont(modeFont);

    const QList<QWidget *> widgets =
        ui->centralwidget->findChildren<QWidget *>(QString(), Qt::FindChildrenRecursively);

    for (QWidget *w : widgets) {
        if (w == nullptr) {
            continue;
        }

        removeFontRulesFromStyleSheet(w);
        w->setFont(modeFont);
    }
}

void saveBaseUiGeometry(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    gBaseCentralSize = ui->centralwidget->size();

    if (gBaseCentralSize.width() <= 0 || gBaseCentralSize.height() <= 0) {
        return;
    }

    const QList<QWidget *> widgets =
        ui->centralwidget->findChildren<QWidget *>(QString(), Qt::FindChildrenRecursively);

    for (QWidget *w : widgets) {
        if (w == nullptr) {
            continue;
        }

        w->setProperty("baseGeometry", w->geometry());

        const qreal fontSize = w->font().pointSizeF();
        if (fontSize > 0.0) {
            w->setProperty("baseFontSize", fontSize);
        }
    }

    gScaleReady = true;
}

void applyUiScale(Ui::MainWindow *ui)
{
    if (gScaleApplying || !gScaleReady || ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    if (gBaseCentralSize.width() <= 0 || gBaseCentralSize.height() <= 0) {
        return;
    }

    gScaleApplying = true;

    const double sx = ui->centralwidget->width() / double(gBaseCentralSize.width());
    const double sy = ui->centralwidget->height() / double(gBaseCentralSize.height());

    // Linear uniform scale: all controls keep the same ratio and do not stretch differently in X/Y.
    const double scale = qMin(sx, sy);

    // Center the scaled UI in the available central widget area.
    const int offsetX = qRound((ui->centralwidget->width() - gBaseCentralSize.width() * scale) / 2.0);
    const int offsetY = qRound((ui->centralwidget->height() - gBaseCentralSize.height() * scale) / 2.0);

    const QList<QWidget *> widgets =
        ui->centralwidget->findChildren<QWidget *>(QString(), Qt::FindChildrenRecursively);

    for (QWidget *w : widgets) {
        if (w == nullptr) {
            continue;
        }

        const QVariant geometryVar = w->property("baseGeometry");
        if (!geometryVar.isValid()) {
            continue;
        }

        const QRect r = geometryVar.toRect();

        const int x = offsetX + qRound(r.x() * scale);
        const int y = offsetY + qRound(r.y() * scale);
        const int width = qMax(1, qRound(r.width() * scale));
        const int height = qMax(1, qRound(r.height() * scale));

        w->setGeometry(x, y, width, height);

        bool fontOk = false;
        const double baseFontSize = w->property("baseFontSize").toDouble(&fontOk);

        if (fontOk && baseFontSize > 0.0) {
            QFont f = w->font();
            f.setPointSizeF(qMax(6.0, baseFontSize * scale));
            w->setFont(f);
        }
    }

    // 右上角注释框已经删除，不再 raise。

    if (gSendAngleStatusLabel != nullptr) {
        gSendAngleStatusLabel->raise();
    }

    if (gSpeedUnitLabel != nullptr) {
        gSpeedUnitLabel->raise();
    }

    if (gConnectionFrame != nullptr) {
        gConnectionFrame->lower();
    }

    if (gLedControlFrame != nullptr) {
        gLedControlFrame->lower();
    }

    if (gStatusLogFrame != nullptr) {
        gStatusLogFrame->lower();
    }

    if (gParameterFrame != nullptr) {
        gParameterFrame->lower();
    }

    if (gAngleControlFrame != nullptr) {
        gAngleControlFrame->lower();
    }

    if (gMiddleImageLabel != nullptr) {
        gMiddleImageLabel->raise();
    }

    if (gOpenUserFileButton != nullptr) {
        gOpenUserFileButton->raise();
    }

    if (gEstopButton != nullptr) {
        gEstopButton->raise();
    }

    if (gRestartEsp32Button != nullptr) {
        gRestartEsp32Button->raise();
    }

    gScaleApplying = false;
}

class UiScaleEventFilter : public QObject
{
public:
    explicit UiScaleEventFilter(Ui::MainWindow *ui, QObject *parent = nullptr)
        : QObject(parent)
        , mUi(ui)
    {
    }

protected:
    bool eventFilter(QObject *watched, QEvent *event) override
    {
        if (event != nullptr && event->type() == QEvent::Resize) {
            QTimer::singleShot(0, this, [this]() {
                applyUiScale(mUi);
            });
        }

        return QObject::eventFilter(watched, event);
    }

private:
    Ui::MainWindow *mUi = nullptr;
};

// Forward declarations for final layout pass used by setupResponsiveScaling().
void positionLedControlAboveEstop(Ui::MainWindow *ui);
void setupLedControlFrame(Ui::MainWindow *ui);
void setupParameterFrame(Ui::MainWindow *ui);
void positionMotorAngleBlockBelowParameter(Ui::MainWindow *ui);
void positionStatusLogLeftAlignedWithWifi(Ui::MainWindow *ui);
void positionSendAngleBlockBottomRight(Ui::MainWindow *ui);
void positionSendAngleStatusLeftOfButton(Ui::MainWindow *ui);
void setupAngleControlFrame(Ui::MainWindow *ui);
void shiftInputColumnsRight(Ui::MainWindow *ui);
void setupMiddleImageInterface(Ui::MainWindow *ui);
void positionMiddleImageBetweenParameterAndLed(Ui::MainWindow *ui);
QRect sendAngleAreaReferenceRect(Ui::MainWindow *ui);


void applyModeFontToWholeUi(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    // Use the same font style as the Mode label. If the Mode label has not
    // been created yet, fall back to the Qt application default UI font.
    QFont modeFont = (gAngleModeLabel != nullptr)
                         ? gAngleModeLabel->font()
                         : QApplication::font();

    // Keep every widget in the same family/style as Mode.
    modeFont.setBold(false);
    modeFont.setItalic(false);

    if (modeFont.pointSize() <= 0) {
        modeFont.setPointSize(9);
    }

    QList<QWidget *> widgets;
    widgets << ui->centralwidget;
    widgets << ui->centralwidget->findChildren<QWidget *>(QString(), Qt::FindChildrenRecursively);

    for (QWidget *w : widgets) {
        if (w == nullptr) {
            continue;
        }

        // Some style sheets had font-size / font-weight rules. Those rules
        // override setFont(), so remove only the font rules and keep colors,
        // borders, background colors, radius, padding, etc.
        removeFontRulesFromStyleSheet(w);
        w->setFont(modeFont);
    }
}



void setupResponsiveScaling(QObject *owner, Ui::MainWindow *ui)
{
    if (owner == nullptr || ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    UiScaleEventFilter *scaleFilter = new UiScaleEventFilter(ui, owner);
    ui->centralwidget->installEventFilter(scaleFilter);

    QTimer::singleShot(0, owner, [ui]() {
        // Now the real window size is known. Put dynamic widgets in the desired base positions first.
        // 右上角注释框已经删除，不再定位显示
        // positionCommTextTopRight(ui->centralwidget);
        setupSpeedUnitLabel(ui);

        // 构造函数刚运行时 centralwidget 的真实宽度可能还没准备好，
        // 所以这里再排一次右侧模块，避免 LED / Save / 参数框相互重叠。
        positionEstopRestartTopRight(ui);
        setupParameterFrame(ui);
        positionMotorAngleBlockBelowParameter(ui);

        // 先确定最下方 Send Angle 整体区域，再让 LED 和 Status Log 的右边框对齐它。
        positionSendAngleBlockBottomRight(ui);
        setupAngleControlFrame(ui);

        positionLedControlAboveEstop(ui);
        setupLedControlFrame(ui);
        setupMiddleImageInterface(ui);
        positionMiddleImageBetweenParameterAndLed(ui);
        positionStatusLogLeftAlignedWithWifi(ui);

        // applyModeFontToAllWidgets(ui);
        applyModeFontToWholeUi(ui);
        saveBaseUiGeometry(ui);
        applyUiScale(ui);
    });
}


// 边框函数
void setupConnectionFrame(Ui::MainWindow *ui)
{
    if (ui == nullptr ||
        ui->centralwidget == nullptr ||
        ui->lineEditIP == nullptr ||
        ui->lineEditPort == nullptr ||
        ui->btnConnect == nullptr ||
        ui->labelConnectStatus == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    gConnectionFrame = parent->findChild<QFrame *>("connectionFrame");

    if (gConnectionFrame == nullptr) {
        gConnectionFrame = new QFrame(parent);
        gConnectionFrame->setObjectName("connectionFrame");
    }

    gConnectionFrame->setStyleSheet(
        "QFrame#connectionFrame {"
        " background-color: transparent;"
        " border: 2px solid #34495e;"
        " border-radius: 10px;"
        "}"
        );

    // 让 Disconnected / Connected 有足够宽度，不被截断
    QRect statusRect = ui->labelConnectStatus->geometry();
    ui->labelConnectStatus->setGeometry(
        statusRect.left(),
        statusRect.top(),
        220,
        statusRect.height()
        );

    // 找到左边的 IP Addr 和 Port 标签
    QLabel *ipLabel = nullptr;
    QLabel *portLabel = nullptr;

    const QList<QLabel *> labels = parent->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        QString text = label->text();
        text.remove('&');

        if (text.contains("IP Addr", Qt::CaseInsensitive)) {
            ipLabel = label;
        }

        if (text.contains("Port", Qt::CaseInsensitive)) {
            portLabel = label;
        }
    }

    const int paddingLeft = 30;
    const int paddingRight = 30;
    const int paddingTop = 25;
    const int paddingBottom = 25;

    int left = ui->lineEditIP->geometry().left();
    int top = ui->lineEditIP->geometry().top();
    int right = ui->lineEditIP->geometry().right();
    int bottom = ui->lineEditIP->geometry().bottom();

    auto includeWidget = [&](QWidget *w) {
        if (w == nullptr) {
            return;
        }

        QRect r = w->geometry();

        left = qMin(left, r.left());
        top = qMin(top, r.top());
        right = qMax(right, r.right());
        bottom = qMax(bottom, r.bottom());
    };

    includeWidget(ipLabel);
    includeWidget(portLabel);
    includeWidget(ui->lineEditIP);
    includeWidget(ui->lineEditPort);
    includeWidget(ui->btnConnect);
    includeWidget(ui->labelConnectStatus);

    gConnectionFrame->setGeometry(
        left - paddingLeft,
        top - paddingTop,
        right - left + paddingLeft + paddingRight,
        bottom - top + paddingTop + paddingBottom
        );

    gConnectionFrame->show();

    // 放到后面，不挡输入框和按钮
    gConnectionFrame->lower();
}

// Wifi模块整体向上移动
void moveConnectionBlockUp(Ui::MainWindow *ui, int dy = 15)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    auto moveUp = [dy](QWidget *w) {
        if (w == nullptr) {
            return;
        }

        QRect r = w->geometry();
        r.moveTop(r.top() - dy);
        w->setGeometry(r);
    };

    // 移动输入框、按钮、状态文字
    moveUp(ui->lineEditIP);
    moveUp(ui->lineEditPort);
    moveUp(ui->btnConnect);
    moveUp(ui->labelConnectStatus);

    // 移动左侧 IP Addr / Port 标签
    const QList<QLabel *> labels = parent->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        QString text = label->text();
        text.remove('&');

        if (text.contains("IP Addr", Qt::CaseInsensitive) ||
            text.contains("Port", Qt::CaseInsensitive)) {
            moveUp(label);
        }
    }
}



void moveStatusLogBlockRight(Ui::MainWindow *ui, int dx = 25)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->textBrowserLog == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    auto moveRight = [dx](QWidget *w) {
        if (w == nullptr) {
            return;
        }

        QRect r = w->geometry();
        r.moveLeft(r.left() + dx);
        w->setGeometry(r);
    };

    // 移动白色日志框
    moveRight(ui->textBrowserLog);

    // 移动 Status Log: 标签
    const QList<QLabel *> labels = parent->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        QString text = label->text();
        text.remove('&');

        if (text.contains("Status Log", Qt::CaseInsensitive)) {
            moveRight(label);
        }
    }
}


void setupStatusLogFrame(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->textBrowserLog == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    gStatusLogFrame = parent->findChild<QFrame *>("statusLogFrame");

    if (gStatusLogFrame == nullptr) {
        gStatusLogFrame = new QFrame(parent);
        gStatusLogFrame->setObjectName("statusLogFrame");
    }

    gStatusLogFrame->setStyleSheet(
        "QFrame#statusLogFrame {"
        " background-color: transparent;"
        " border: 2px solid #34495e;"
        " border-radius: 10px;"
        "}"
        );

    QLabel *statusLogLabel = nullptr;

    const QList<QLabel *> labels = parent->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        QString text = label->text();
        text.remove('&');

        if (text.contains("Status Log", Qt::CaseInsensitive)) {
            statusLogLabel = label;
            break;
        }
    }

    const int paddingLeft = 12;
    const int paddingRight = 12;
    const int paddingTop = 12;
    const int paddingBottom = 12;

    int left = ui->textBrowserLog->geometry().left();
    int top = ui->textBrowserLog->geometry().top();
    int right = ui->textBrowserLog->geometry().right();
    int bottom = ui->textBrowserLog->geometry().bottom();

    auto includeWidget = [&](QWidget *w) {
        if (w == nullptr) {
            return;
        }

        QRect r = w->geometry();

        left = qMin(left, r.left());
        top = qMin(top, r.top());
        right = qMax(right, r.right());
        bottom = qMax(bottom, r.bottom());
    };

    includeWidget(statusLogLabel);
    includeWidget(ui->textBrowserLog);

    gStatusLogFrame->setGeometry(
        left - paddingLeft,
        top - paddingTop,
        right - left + paddingLeft + paddingRight,
        bottom - top + paddingTop + paddingBottom
        );

    gStatusLogFrame->show();
    gStatusLogFrame->lower();

    if (statusLogLabel != nullptr) {
        statusLogLabel->raise();
    }


    ui->textBrowserLog->raise();
}

QLabel *findLabelContainsText(QWidget *parent, const QStringList &keywords)
{
    if (parent == nullptr) {
        return nullptr;
    }

    const QList<QLabel *> labels = parent->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        if (label == nullptr) {
            continue;
        }

        QString text = label->text();
        text.remove('&');

        for (const QString &keyword : keywords) {
            if (text.contains(keyword, Qt::CaseInsensitive)) {
                return label;
            }
        }
    }

    return nullptr;
}

QLabel *findLabelExactText(QWidget *parent, const QString &keyword)
{
    if (parent == nullptr) {
        return nullptr;
    }

    const QList<QLabel *> labels = parent->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        if (label == nullptr) {
            continue;
        }

        QString text = label->text();
        text.remove('&');
        text.remove(':');
        text = text.trimmed();

        if (text.compare(keyword, Qt::CaseInsensitive) == 0) {
            return label;
        }
    }

    return nullptr;
}

void removeFontRulesFromStyleSheet(QWidget *w)
{
    if (w == nullptr) {
        return;
    }

    QString ss = w->styleSheet();

    if (ss.trimmed().isEmpty()) {
        return;
    }

    ss.remove(QRegularExpression("font-family\\s*:[^;]+;"));
    ss.remove(QRegularExpression("font-size\\s*:[^;]+;"));
    ss.remove(QRegularExpression("font-weight\\s*:[^;]+;"));
    ss.remove(QRegularExpression("font-style\\s*:[^;]+;"));

    w->setStyleSheet(ss);
}


QList<QWidget *> ledControlWidgets(Ui::MainWindow *ui)
{
    QList<QWidget *> widgets;

    if (ui == nullptr || ui->centralwidget == nullptr) {
        return widgets;
    }

    auto addWidget = [&](QWidget *w) {
        if (w != nullptr && !widgets.contains(w)) {
            widgets.append(w);
        }
    };

    addWidget(ui->btnLedOn);
    // LED ON / LED OFF now share one toggle button, so the old OFF button is hidden
    // and should not affect the LED control frame size or layout.
    addWidget(ui->sliderR);
    addWidget(ui->sliderG);
    addWidget(ui->sliderB);
    addWidget(ui->colorPreview);

    const QList<QLabel *> labels = ui->centralwidget->findChildren<QLabel *>();

    for (QLabel *label : labels) {
        QString text = label->text().trimmed();
        text.remove('&');

        if (text == "R" ||
            text == "G" ||
            text == "B" ||
            text.contains("Color Preview", Qt::CaseInsensitive)) {
            addWidget(label);
        }
    }

    return widgets;
}

QRect boundingRectForWidgets(const QList<QWidget *> &widgets)
{
    QRect result;
    bool initialized = false;

    for (QWidget *w : widgets) {
        if (w == nullptr) {
            continue;
        }

        const QRect r = w->geometry();

        if (!initialized) {
            result = r;
            initialized = true;
        } else {
            result = result.united(r);
        }
    }

    return result;
}


int rightAlignTargetEdge(Ui::MainWindow *ui)
{
    if (gEstopButton != nullptr && gEstopButton->geometry().isValid()) {
        return gEstopButton->geometry().right();
    }

    if (ui != nullptr && ui->centralwidget != nullptr && ui->centralwidget->width() > 0) {
        return ui->centralwidget->width() - 35;
    }

    return 1380;
}

QRect sendAngleAreaReferenceRect(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return QRect();
    }

    // 优先使用最下方整个 Send Angle / Motor Angle 外框。
    // 这样 LED 区域和 Status Log 区域都能以同一个右边界为基准。
    if (gAngleControlFrame != nullptr && gAngleControlFrame->geometry().isValid()) {
        return gAngleControlFrame->geometry();
    }

    QWidget *parent = ui->centralwidget;
    QList<QWidget *> widgets;

    auto addWidget = [&](QWidget *w) {
        if (w != nullptr && !widgets.contains(w)) {
            widgets.append(w);
        }
    };

    addWidget(findLabelContainsText(parent, QStringList() << "Motor Angle"));
    addWidget(findLabelContainsText(parent, QStringList() << "Current Angle"));
    addWidget(ui->angleEdit);
    addWidget(ui->currentAngleEdit);
    addWidget(gResetAngleButton);
    addWidget(ui->sendAngleButton);
    addWidget(gAngleModeLabel);
    addWidget(gAngleModeCombo);

    const QRect contentRect = boundingRectForWidgets(widgets);

    if (!contentRect.isValid()) {
        return QRect();
    }

    const int paddingLeft = 28;
    const int paddingRight = 28;
    const int paddingTop = 22;
    const int paddingBottom = 22;

    return QRect(
        contentRect.left() - paddingLeft,
        contentRect.top() - paddingTop,
        contentRect.width() + paddingLeft + paddingRight,
        contentRect.height() + paddingTop + paddingBottom
        );
}

void moveWidgetsBy(const QList<QWidget *> &widgets, int dx, int dy)
{
    for (QWidget *w : widgets) {
        if (w == nullptr) {
            continue;
        }

        QRect r = w->geometry();
        r.translate(dx, dy);
        w->setGeometry(r);
    }
}

void setupLedControlFrame(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;
    const QList<QWidget *> widgets = ledControlWidgets(ui);
    const QRect contentRect = boundingRectForWidgets(widgets);

    if (!contentRect.isValid()) {
        return;
    }

    gLedControlFrame = parent->findChild<QFrame *>("ledControlFrame");

    if (gLedControlFrame == nullptr) {
        gLedControlFrame = new QFrame(parent);
        gLedControlFrame->setObjectName("ledControlFrame");
    }

    gLedControlFrame->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    gLedControlFrame->setStyleSheet(
        "QFrame#ledControlFrame {"
        " background-color: transparent;"
        " border: 2px solid #34495e;"
        " border-radius: 10px;"
        "}"
        );

    const int paddingLeft = 25;
    const int paddingRight = 25;
    const int paddingTop = 25;
    const int paddingBottom = 25;

    const QRect parameterRect = (gParameterFrame != nullptr)
                                    ? gParameterFrame->geometry()
                                    : QRect();

    if (parameterRect.isValid()) {
        // LED 模块右边和 ESTOP 对齐，同时与参数模块顶部/底部对齐。
        const int targetFrameRight = rightAlignTargetEdge(ui);
        const int frameWidth = qMax(390, contentRect.width() + paddingLeft + paddingRight);
        gLedControlFrame->setGeometry(
            targetFrameRight - frameWidth + 1,
            parameterRect.top(),
            frameWidth,
            parameterRect.height()
            );
    } else {
        gLedControlFrame->setGeometry(
            contentRect.left() - paddingLeft,
            contentRect.top() - paddingTop,
            contentRect.width() + paddingLeft + paddingRight,
            contentRect.height() + paddingTop + paddingBottom
            );
    }

    gLedControlFrame->show();
    gLedControlFrame->lower();

    for (QWidget *w : widgets) {
        if (w != nullptr) {
            w->raise();
        }
    }
}

void refreshMiddleImagePixmap()
{
    if (gMiddleImageLabel == nullptr) {
        return;
    }

    const QString imagePath = gMiddleImageLabel->property("middleImagePath").toString();
    const QFileInfo imageInfo(imagePath);

    if (!imagePath.isEmpty() && imageInfo.exists() && imageInfo.isFile()) {
        QPixmap pixmap(imageInfo.absoluteFilePath());

        if (!pixmap.isNull()) {
            gMiddleImageLabel->setText("");
            gMiddleImageLabel->setStyleSheet(
                "QLabel#middleImageLabel {"
                " background-color: white;"
                " border: 2px solid #34495e;"
                " border-radius: 8px;"
                "}"
                );
            gMiddleImageLabel->setPixmap(pixmap);
            return;
        }
    }

    // 没有图片文件时显示占位框。之后把 middle_image.png 放到 exe 同目录即可自动显示。
    gMiddleImageLabel->setPixmap(QPixmap());
    gMiddleImageLabel->setText("IMAGE\nmiddle_image.png");
    gMiddleImageLabel->setStyleSheet(
        "QLabel#middleImageLabel {"
        " background-color: #ffffff;"
        " color: #7f8c8d;"
        " border: 2px dashed #95a5a6;"
        " border-radius: 8px;"
        "}"
        );
}

void setupMiddleImageInterface(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    gMiddleImageLabel = parent->findChild<QLabel *>("middleImageLabel");

    if (gMiddleImageLabel == nullptr) {
        gMiddleImageLabel = new QLabel(parent);
        gMiddleImageLabel->setObjectName("middleImageLabel");
    }

    gMiddleImageLabel->setAlignment(Qt::AlignCenter);
    gMiddleImageLabel->setWordWrap(true);
    gMiddleImageLabel->setScaledContents(true);
    gMiddleImageLabel->setToolTip(
        "Image interface: put middle_image.png in the same folder as the executable, "
        "or change the middleImagePath property in code."
        );

    // 默认接口：程序 exe 同目录下放 middle_image.png 即可显示。
    const QString defaultImagePath = QCoreApplication::applicationDirPath() + "/middle_image.png";
    gMiddleImageLabel->setProperty("middleImagePath", defaultImagePath);

    refreshMiddleImagePixmap();
    gMiddleImageLabel->show();
    gMiddleImageLabel->raise();
}

void positionMiddleImageBetweenParameterAndLed(Ui::MainWindow *ui)
{
    if (ui == nullptr ||
        ui->centralwidget == nullptr ||
        gMiddleImageLabel == nullptr ||
        gParameterFrame == nullptr ||
        gLedControlFrame == nullptr) {
        return;
    }

    const QRect parameterRect = gParameterFrame->geometry();
    const QRect ledRect = gLedControlFrame->geometry();

    if (!parameterRect.isValid() || !ledRect.isValid()) {
        return;
    }

    const int sideGap = 22;
    const int imageLeft = parameterRect.right() + sideGap;
    const int imageRight = ledRect.left() - sideGap;

    if (imageRight <= imageLeft + 40) {
        gMiddleImageLabel->hide();
        return;
    }

    // 图片接口和参数模块 / LED 模块上下对齐，中间左右留一点空隙。
    gMiddleImageLabel->setGeometry(
        imageLeft,
        parameterRect.top(),
        imageRight - imageLeft + 1,
        parameterRect.height()
        );

    refreshMiddleImagePixmap();
    gMiddleImageLabel->show();
    gMiddleImageLabel->raise();
}



void positionLedControlAboveEstop(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->btnLedOn == nullptr) {
        return;
    }

    const int paddingLeft = 25;
    const int paddingRight = 25;
    const int paddingTop = 25;

    const QRect parameterRect = (gParameterFrame != nullptr)
                                    ? gParameterFrame->geometry()
                                    : QRect(20, 480, 700, 280);

    // LED 模块整体向右对齐到 ESTOP 的右边。
    const int targetFrameRight = rightAlignTargetEdge(ui);
    const int frameWidth = 390;
    const int targetFrameLeft = targetFrameRight - frameWidth + 1;

    // LED 模块和参数模块上下对齐：顶部一致，外框高度在 setupLedControlFrame() 中保持一致。
    const int targetFrameTop = parameterRect.top();

    const int contentLeft = targetFrameLeft + paddingLeft;
    const int contentTop = targetFrameTop + paddingTop;
    const int contentRight = targetFrameRight - paddingRight;

    const int buttonWidth = qMax(110, ui->btnLedOn->width());
    const int buttonHeight = qMax(50, ui->btnLedOn->height());

    // LED ON / LED OFF 合并按钮放到 LED 模块右上角。
    ui->btnLedOn->setGeometry(
        contentRight - buttonWidth + 1,
        contentTop,
        buttonWidth,
        buttonHeight
        );
    ui->btnLedOn->show();
    ui->btnLedOn->raise();

    if (ui->btnLedOff != nullptr) {
        ui->btnLedOff->hide();
        ui->btnLedOff->setEnabled(false);
    }

    QLabel *rLabel = findLabelExactText(ui->centralwidget, "R");
    QLabel *gLabel = findLabelExactText(ui->centralwidget, "G");
    QLabel *bLabel = findLabelExactText(ui->centralwidget, "B");

    const int labelWidth = 28;
    const int labelHeight = 24;
    const int sliderHeight = (ui->sliderR != nullptr && ui->sliderR->height() > 0)
                                 ? ui->sliderR->height()
                                 : 22;
    const int sliderX = contentLeft + labelWidth + 8;
    const int sliderWidth = qMax(130, ui->btnLedOn->geometry().left() - sliderX - 28);
    const int rowGap = 38;
    const int firstRowY = contentTop + 10;

    auto placeLedRow = [&](QLabel *label, QSlider *slider, int row) {
        const int y = firstRowY + row * rowGap;

        if (label != nullptr) {
            label->setGeometry(contentLeft, y - 2, labelWidth, labelHeight);
            label->show();
            label->raise();
        }

        if (slider != nullptr) {
            slider->setGeometry(sliderX, y, sliderWidth, sliderHeight);
            slider->show();
            slider->raise();
        }
    };

    placeLedRow(rLabel, ui->sliderR, 0);
    placeLedRow(gLabel, ui->sliderG, 1);
    placeLedRow(bLabel, ui->sliderB, 2);

    if (ui->colorPreview != nullptr) {
        const int previewSize = qMax(40, qMax(ui->colorPreview->width(), ui->colorPreview->height()));
        const int previewX = sliderX + (sliderWidth - previewSize) / 2;
        const int previewY = firstRowY + rowGap * 3 + 15;

        ui->colorPreview->setGeometry(previewX, previewY, previewSize, previewSize);
        ui->colorPreview->show();
        ui->colorPreview->raise();
    }
}

void positionSendAngleStatusLeftOfButton(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->sendAngleButton == nullptr || gSendAngleStatusLabel == nullptr) {
        return;
    }

    const QRect buttonRect = ui->sendAngleButton->geometry();

    const int labelWidth = 160;
    const int labelHeight = 45;
    const int gap = 25;

    int x = buttonRect.left() - gap - labelWidth;
    int y = buttonRect.top() + (buttonRect.height() - labelHeight) / 2;

    if (x < 10) {
        x = 10;
    }

    gSendAngleStatusLabel->setGeometry(x, y, labelWidth, labelHeight);
    gSendAngleStatusLabel->show();
    gSendAngleStatusLabel->raise();
}


void positionSendAngleBlockBottomRight(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->sendAngleButton == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    const QRect currentButtonRect = ui->sendAngleButton->geometry();
    const int buttonWidth = qMax(250, currentButtonRect.width());
    const int buttonHeight = qMax(90, currentButtonRect.height());

    // Send Angle 模块往右下角移动一点。
    const int marginRight = 260;
    int buttonX = parent->width() - marginRight - buttonWidth;

    if (gParameterFrame != nullptr) {
        buttonX = qMax(buttonX, gParameterFrame->geometry().right() + 120);
    }

    if (buttonX + buttonWidth > parent->width() - 20) {
        buttonX = parent->width() - buttonWidth - 20;
    }

    if (buttonX < 20) {
        buttonX = 20;
    }

    int buttonY = currentButtonRect.top() + 45;

    if (gParameterFrame != nullptr) {
        buttonY = qMax(buttonY, gParameterFrame->geometry().bottom() + 24);
    }

    const int comboHeight = (gAngleModeCombo != nullptr && gAngleModeCombo->height() > 0)
                                ? gAngleModeCombo->height()
                                : 28;
    const int bottomLimit = parent->height() - buttonHeight - comboHeight - 35;

    if (bottomLimit > 0 && buttonY > bottomLimit) {
        buttonY = bottomLimit;
    }

    ui->sendAngleButton->setGeometry(buttonX, buttonY, buttonWidth, buttonHeight);
    ui->sendAngleButton->show();
    ui->sendAngleButton->raise();
    positionSendAngleStatusLeftOfButton(ui);

    const int comboY = buttonY + buttonHeight + 10;
    const int labelWidth = 70;
    // 下拉框调小一点，保持够用但不占太宽。
    const int comboWidth = (gAngleModeCombo != nullptr && gAngleModeCombo->width() > 0)
                               ? qMax(300, qMin(gAngleModeCombo->width(), 340))
                               : 300;

    if (gAngleModeLabel != nullptr) {
        gAngleModeLabel->setGeometry(buttonX, comboY, labelWidth, comboHeight);
        gAngleModeLabel->show();
        gAngleModeLabel->raise();
    }

    if (gAngleModeCombo != nullptr) {
        gAngleModeCombo->setGeometry(buttonX + labelWidth + 8, comboY, comboWidth, comboHeight);
        gAngleModeCombo->show();
        gAngleModeCombo->raise();
    }
}



void setupAngleControlFrame(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->sendAngleButton == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    QList<QWidget *> widgets;

    auto addWidget = [&](QWidget *w) {
        if (w != nullptr && !widgets.contains(w)) {
            widgets.append(w);
        }
    };

    addWidget(findLabelContainsText(parent, QStringList() << "Motor Angle"));
    addWidget(findLabelContainsText(parent, QStringList() << "Current Angle"));
    addWidget(ui->angleEdit);
    addWidget(ui->currentAngleEdit);
    addWidget(gResetAngleButton);
    addWidget(ui->sendAngleButton);
    addWidget(gAngleModeLabel);
    addWidget(gAngleModeCombo);

    const QRect contentRect = boundingRectForWidgets(widgets);

    if (!contentRect.isValid()) {
        return;
    }

    gAngleControlFrame = parent->findChild<QFrame *>("angleControlFrame");

    if (gAngleControlFrame == nullptr) {
        gAngleControlFrame = new QFrame(parent);
        gAngleControlFrame->setObjectName("angleControlFrame");
    }

    gAngleControlFrame->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    gAngleControlFrame->setStyleSheet(
        "QFrame#angleControlFrame {"
        " background-color: transparent;"
        " border: 2px solid #34495e;"
        " border-radius: 10px;"
        "}"
        );

    const int paddingLeft = 28;
    const int paddingRight = 28;
    const int paddingTop = 22;
    const int paddingBottom = 22;

    const int frameLeft = contentRect.left() - paddingLeft;
    const int frameTop = contentRect.top() - paddingTop;
    const int naturalFrameRight = contentRect.right() + paddingRight;
    const int targetFrameRight = qMax(naturalFrameRight, rightAlignTargetEdge(ui));

    // Send Angle 模块向右拉长，右边和 LED / ESTOP 对齐。
    gAngleControlFrame->setGeometry(
        frameLeft,
        frameTop,
        targetFrameRight - frameLeft + 1,
        contentRect.height() + paddingTop + paddingBottom
        );

    gAngleControlFrame->show();
    gAngleControlFrame->lower();

    for (QWidget *w : widgets) {
        if (w != nullptr) {
            w->raise();
        }
    }
}

void positionStatusLogLeftAlignedWithWifi(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->textBrowserLog == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;
    QLabel *statusLogLabel = findLabelContainsText(parent, QStringList() << "Status Log");

    const QRect wifiRect = (gConnectionFrame != nullptr)
                               ? gConnectionFrame->geometry()
                               : QRect(20, 80, 800, 140);

    const int gapBetweenModules = 16;
    const int padding = 12;
    const int labelToLogGap = 8;

    // 左边保持和 Wi-Fi 区域对齐，右边拉长到 ESTOP 的右边。
    const int frameLeft = wifiRect.left();

    int frameRight = rightAlignTargetEdge(ui);

    if (frameRight - frameLeft < 420) {
        frameRight = wifiRect.right();
    }

    const int frameTop = wifiRect.bottom() + gapBetweenModules;
    const int frameHeight = qMax(210, ui->textBrowserLog->height() + 60);
    const int frameWidth = frameRight - frameLeft + 1;
    const int innerLeft = frameLeft + padding;
    const int innerWidth = qMax(320, frameWidth - padding * 2);
    const int labelHeight = (statusLogLabel != nullptr && statusLogLabel->height() > 0)
                                ? statusLogLabel->height()
                                : 24;

    const int labelTop = frameTop + padding;
    const int logTop = labelTop + labelHeight + labelToLogGap;
    const int logHeight = qMax(120, frameTop + frameHeight - padding - logTop);

    if (statusLogLabel != nullptr) {
        statusLogLabel->setGeometry(innerLeft, labelTop, innerWidth, labelHeight);
        statusLogLabel->show();
        statusLogLabel->raise();
    }

    ui->textBrowserLog->setGeometry(innerLeft, logTop, innerWidth, logHeight);
    ui->textBrowserLog->show();
    ui->textBrowserLog->raise();

    setupStatusLogFrame(ui);
}


void setupParameterLabelFont(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    QList<QLabel *> labels;
    labels << findLabelExactText(parent, "KP");
    labels << findLabelExactText(parent, "KI");
    labels << findLabelExactText(parent, "KD");
    labels << findLabelContainsText(parent, QStringList() << "Speed");
    labels << (gAbsAngleLimitLabel != nullptr
                   ? gAbsAngleLimitLabel
                   : findLabelContainsText(parent, QStringList() << "Abs Angle"));

    // 用 Mode 的字体作为标准
    QFont modeFont;

    if (gAngleModeLabel != nullptr) {
        modeFont = gAngleModeLabel->font();
    } else {
        modeFont = QApplication::font();
    }

    modeFont.setBold(false);
    modeFont.setItalic(false);
    // Do not force point size; keep it exactly the same as Mode.

    for (QLabel *label : labels) {
        if (label == nullptr) {
            continue;
        }

        label->setFont(modeFont);

        label->setStyleSheet(
            "QLabel {"
            " color: black;"
            "}"
            );
    }
}

void arrangeSpeedAndAngleLimitUnderKd(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->kdEdit == nullptr || ui->speedEdit == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    QLabel *kdLabel = findLabelExactText(parent, "KD");
    QLabel *speedLabel = findLabelContainsText(parent, QStringList() << "Speed");
    QLabel *absLabel = (gAbsAngleLimitLabel != nullptr)
                           ? gAbsAngleLimitLabel
                           : findLabelContainsText(parent, QStringList() << "Abs Angle");

    const QRect kdRect = ui->kdEdit->geometry();
    const int editX = kdRect.left();
    const int editWidth = kdRect.width();
    const int editHeight = kdRect.height() > 0 ? kdRect.height() : 32;

    int labelX = (kdLabel != nullptr) ? kdLabel->geometry().left() : qMax(10, editX - 150);

    // Abs Angle Limit 文字比较长，给标签留足宽度，避免被截断。
    if (editX - labelX < 145) {
        labelX = qMax(10, editX - 160);
    }

    const int labelWidth = qMax(110, editX - labelX - 12);
    const int rowGap = 10;
    const int sectionGap = 14;

    const int speedY = kdRect.bottom() + sectionGap;
    const int absY = speedY + editHeight + rowGap;

    if (speedLabel != nullptr) {
        speedLabel->setGeometry(labelX, speedY, labelWidth, editHeight);
        speedLabel->show();
        speedLabel->raise();
    }

    ui->speedEdit->setGeometry(editX, speedY, editWidth, editHeight);
    ui->speedEdit->show();
    ui->speedEdit->raise();

    if (gSpeedUnitLabel != nullptr) {
        gSpeedUnitLabel->setGeometry(editX + editWidth + 12, speedY, 90, editHeight);
        gSpeedUnitLabel->show();
        gSpeedUnitLabel->raise();
    }

    if (absLabel != nullptr) {
        absLabel->setGeometry(labelX, absY, labelWidth, editHeight);
        absLabel->show();
        absLabel->raise();
    }

    if (gAbsAngleLimitEdit != nullptr) {
        gAbsAngleLimitEdit->setGeometry(editX, absY, editWidth, editHeight);
        gAbsAngleLimitEdit->show();
        gAbsAngleLimitEdit->raise();
    }
}

void shiftInputColumnsRight(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    // 给左侧标签留更多空间：Current Angle 不会再被输入框挡住。
    // 只执行一次，避免 resize / 二次排版时重复右移。
    const int shiftRightPx = 70;

    auto moveOnce = [shiftRightPx](QWidget *w) {
        if (w == nullptr || w->property("inputColumnShiftedRight").toBool()) {
            return;
        }

        QRect r = w->geometry();
        r.translate(shiftRightPx, 0);
        w->setGeometry(r);
        w->setProperty("inputColumnShiftedRight", true);
    };

    moveOnce(ui->kpEdit);
    moveOnce(ui->kiEdit);
    moveOnce(ui->kdEdit);
    moveOnce(ui->speedEdit);
    moveOnce(gAbsAngleLimitEdit);
    moveOnce(gSpeedUnitLabel);

    // Motor Angle / Current Angle 的输入框也右移，给标签留完整显示宽度。
    moveOnce(ui->angleEdit);
    moveOnce(ui->currentAngleEdit);

    QLabel *motorAngleLabel =
        findLabelContainsText(ui->centralwidget, QStringList() << "Motor Angle");
    QLabel *currentAngleLabel =
        findLabelContainsText(ui->centralwidget, QStringList() << "Current Angle");

    // 让参数模块左侧文字和 Motor Angle / Current Angle 左侧文字对齐。
    // 输入框已经右移，所以这里不再动输入框，只把 KP/KI/KD/Speed/Angle Limit 标签左对齐。
    const int alignedLabelX = (motorAngleLabel != nullptr)
                                  ? motorAngleLabel->geometry().left()
                                  : 56;

    auto alignParameterLabelBeforeEdit = [alignedLabelX](QLabel *label, QLineEdit *edit) {
        if (label == nullptr || edit == nullptr) {
            return;
        }

        QRect labelRect = label->geometry();
        labelRect.moveLeft(alignedLabelX);
        labelRect.setWidth(qMax(labelRect.width(), edit->geometry().left() - alignedLabelX - 12));
        label->setGeometry(labelRect);
        label->show();
        label->raise();
    };

    alignParameterLabelBeforeEdit(findLabelExactText(ui->centralwidget, "KP"), ui->kpEdit);
    alignParameterLabelBeforeEdit(findLabelExactText(ui->centralwidget, "KI"), ui->kiEdit);
    alignParameterLabelBeforeEdit(findLabelExactText(ui->centralwidget, "KD"), ui->kdEdit);
    alignParameterLabelBeforeEdit(findLabelContainsText(ui->centralwidget, QStringList() << "Speed"), ui->speedEdit);
    alignParameterLabelBeforeEdit(gAbsAngleLimitLabel != nullptr
                                      ? gAbsAngleLimitLabel
                                      : findLabelContainsText(ui->centralwidget, QStringList() << "Abs Angle"),
                                  gAbsAngleLimitEdit);

    auto widenLabelBeforeEdit = [](QLabel *label, QLineEdit *edit) {
        if (label == nullptr || edit == nullptr) {
            return;
        }

        QRect labelRect = label->geometry();
        const int availableWidth = edit->geometry().left() - labelRect.left() - 12;

        if (availableWidth > labelRect.width()) {
            labelRect.setWidth(availableWidth);
            label->setGeometry(labelRect);
        }

        label->show();
        label->raise();
    };

    widenLabelBeforeEdit(motorAngleLabel, ui->angleEdit);
    widenLabelBeforeEdit(currentAngleLabel, ui->currentAngleEdit);
}



void positionParameterBlockUpAndSaveButton(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;
    QList<QWidget *> parameterWidgets;

    auto addWidget = [&](QWidget *w) {
        if (w != nullptr && !parameterWidgets.contains(w)) {
            parameterWidgets.append(w);
        }
    };

    addWidget(findLabelExactText(parent, "KP"));
    addWidget(findLabelExactText(parent, "KI"));
    addWidget(findLabelExactText(parent, "KD"));
    addWidget(findLabelContainsText(parent, QStringList() << "Speed"));
    addWidget(gAbsAngleLimitLabel != nullptr ? gAbsAngleLimitLabel : findLabelContainsText(parent, QStringList() << "Abs Angle"));
    addWidget(ui->kpEdit);
    addWidget(ui->kiEdit);
    addWidget(ui->kdEdit);
    addWidget(ui->speedEdit);
    addWidget(gAbsAngleLimitEdit);
    addWidget(gSpeedUnitLabel);

    // 整个参数区域稍微上移一点。
    const int moveUp = 35;
    moveWidgetsBy(parameterWidgets, 0, -moveUp);

    const QRect contentRect = boundingRectForWidgets(parameterWidgets);
    if (!contentRect.isValid() || ui->btnSaveParameter == nullptr) {
        return;
    }

    // Save 放进同一个参数框里，大小和 LED ON 按钮保持一致。
    const QSize saveButtonSize = (ui->btnLedOn != nullptr && ui->btnLedOn->width() > 0 && ui->btnLedOn->height() > 0)
                                     ? ui->btnLedOn->size()
                                     : QSize(110, 50);
    const int buttonWidth = saveButtonSize.width();
    const int buttonHeight = saveButtonSize.height();
    const int gap = 35;

    const int buttonX = contentRect.right() + gap;
    // Save 放到参数模块右上角：和第一行参数顶部对齐，不再垂直居中。
    const int buttonY = contentRect.top();

    ui->btnSaveParameter->setGeometry(buttonX, buttonY, buttonWidth, buttonHeight);
    ui->btnSaveParameter->setMinimumSize(buttonWidth, buttonHeight);
    ui->btnSaveParameter->setMaximumSize(buttonWidth, buttonHeight);
    ui->btnSaveParameter->setText("Save");

    ui->btnSaveParameter->setStyleSheet(GREEN_BUTTON_STYLE);
    ui->btnSaveParameter->show();
    ui->btnSaveParameter->raise();
}

void setupParameterFrame(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;

    gParameterFrame = parent->findChild<QFrame *>("parameterFrame");

    if (gParameterFrame == nullptr) {
        gParameterFrame = new QFrame(parent);
        gParameterFrame->setObjectName("parameterFrame");
    }

    gParameterFrame->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    gParameterFrame->setStyleSheet(
        "QFrame#parameterFrame {"
        " background-color: transparent;"
        " border: 2px solid #34495e;"
        " border-radius: 10px;"
        "}"
        );

    QList<QWidget *> widgets;

    auto addWidget = [&](QWidget *w) {
        if (w != nullptr && !widgets.contains(w)) {
            widgets.append(w);
        }
    };

    addWidget(findLabelExactText(parent, "KP"));
    addWidget(findLabelExactText(parent, "KI"));
    addWidget(findLabelExactText(parent, "KD"));
    addWidget(findLabelContainsText(parent, QStringList() << "Speed"));
    addWidget(gAbsAngleLimitLabel != nullptr ? gAbsAngleLimitLabel : findLabelContainsText(parent, QStringList() << "Abs Angle"));
    addWidget(ui->kpEdit);
    addWidget(ui->kiEdit);
    addWidget(ui->kdEdit);
    addWidget(ui->speedEdit);
    addWidget(gAbsAngleLimitEdit);
    addWidget(gSpeedUnitLabel);
    addWidget(ui->btnSaveParameter);

    const QRect contentRect = boundingRectForWidgets(widgets);

    if (!contentRect.isValid()) {
        return;
    }

    const int paddingLeft = 28;
    const int paddingRight = 28;
    const int paddingTop = 22;
    const int paddingBottom = 22;

    int frameLeft = contentRect.left() - paddingLeft;
    QLabel *motorAngleLabel = findLabelContainsText(parent, QStringList() << "Motor Angle");

    if (motorAngleLabel != nullptr) {
        frameLeft = qMin(frameLeft, motorAngleLabel->geometry().left() - paddingLeft);
    }

    const int frameRight = contentRect.right() + paddingRight;

    gParameterFrame->setGeometry(
        frameLeft,
        contentRect.top() - paddingTop,
        frameRight - frameLeft + 1,
        contentRect.height() + paddingTop + paddingBottom
        );

    gParameterFrame->show();
    gParameterFrame->lower();

    for (QWidget *w : widgets) {
        if (w != nullptr) {
            w->raise();
        }
    }
}

void positionMotorAngleBlockBelowParameter(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr || ui->angleEdit == nullptr || ui->currentAngleEdit == nullptr) {
        return;
    }

    QWidget *parent = ui->centralwidget;
    QList<QWidget *> widgets;

    auto addWidget = [&](QWidget *w) {
        if (w != nullptr && !widgets.contains(w)) {
            widgets.append(w);
        }
    };

    addWidget(findLabelContainsText(parent, QStringList() << "Motor Angle"));
    addWidget(findLabelContainsText(parent, QStringList() << "Current Angle"));
    addWidget(ui->angleEdit);
    addWidget(ui->currentAngleEdit);
    addWidget(gResetAngleButton);

    const QRect contentRect = boundingRectForWidgets(widgets);

    if (!contentRect.isValid()) {
        return;
    }

    const QRect parameterRect = (gParameterFrame != nullptr)
                                    ? gParameterFrame->geometry()
                                    : QRect(contentRect.left(), 520, contentRect.width(), 260);

    // 缩小参数模块和 Send Angle 模块的垂直距离，保证底部完整显示。
    const int gap = 30;
    const int targetTop = parameterRect.bottom() + gap;

    moveWidgetsBy(widgets, 0, targetTop - contentRect.top());

    for (QWidget *w : widgets) {
        if (w != nullptr) {
            w->show();
            w->raise();
        }
    }
}


void fineTuneParameterAndLedLayout(Ui::MainWindow *ui)
{
    if (ui == nullptr || ui->centralwidget == nullptr) {
        return;
    }

    const int moveUpPx = 30;        // 参数区和 LED 区整体往上移动，数字越大越往上
    const int moveLedLeftPx = 4000;   // LED 区往左靠近参数区，数字越大越靠近

    QList<QWidget *> parameterWidgets;

    auto addParameterWidget = [&](QWidget *w) {
        if (w != nullptr && !parameterWidgets.contains(w)) {
            parameterWidgets.append(w);
        }
    };

    QWidget *parent = ui->centralwidget;

    addParameterWidget(gParameterFrame);
    addParameterWidget(findLabelExactText(parent, "KP"));
    addParameterWidget(findLabelExactText(parent, "KI"));
    addParameterWidget(findLabelExactText(parent, "KD"));
    addParameterWidget(findLabelContainsText(parent, QStringList() << "Speed"));
    addParameterWidget(gAbsAngleLimitLabel != nullptr
                           ? gAbsAngleLimitLabel
                           : findLabelContainsText(parent, QStringList() << "Angle Limit"));

    addParameterWidget(ui->kpEdit);
    addParameterWidget(ui->kiEdit);
    addParameterWidget(ui->kdEdit);
    addParameterWidget(ui->speedEdit);
    addParameterWidget(gAbsAngleLimitEdit);
    addParameterWidget(gSpeedUnitLabel);
    addParameterWidget(ui->btnSaveParameter);

    // 参数区域整体往上
    moveWidgetsBy(parameterWidgets, 0, -moveUpPx);

    QList<QWidget *> ledWidgets = ledControlWidgets(ui);

    if (gLedControlFrame != nullptr && !ledWidgets.contains(gLedControlFrame)) {
        ledWidgets.append(gLedControlFrame);
    }

    // LED 区域：往上，同时往左靠近参数区
    moveWidgetsBy(ledWidgets, -moveLedLeftPx, -moveUpPx);

    if (gParameterFrame != nullptr) {
        gParameterFrame->lower();
    }

    if (gLedControlFrame != nullptr) {
        gLedControlFrame->lower();
    }

    for (QWidget *w : parameterWidgets) {
        if (w != nullptr && w != gParameterFrame) {
            w->raise();
        }
    }

    for (QWidget *w : ledWidgets) {
        if (w != nullptr && w != gLedControlFrame) {
            w->raise();
        }
    }
}
void moveSendAngleBlockLeftUp(Ui::MainWindow *ui)
{
    if (ui == nullptr) {
        return;
    }

    const int moveLeftPx = 40;  // 数字越大，越往左
    const int moveUpPx = 50;    // 数字越大，越往上

    QList<QWidget *> widgets;

    auto addWidget = [&](QWidget *w) {
        if (w != nullptr && !widgets.contains(w)) {
            widgets.append(w);
        }
    };

    addWidget(ui->sendAngleButton);
    addWidget(gAngleModeLabel);
    addWidget(gAngleModeCombo);

    moveWidgetsBy(widgets, -moveLeftPx, -moveUpPx);

    for (QWidget *w : widgets) {
        if (w != nullptr) {
            w->show();
            w->raise();
        }
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

    // 给新布局多一点高度，避免底部 Send Angle / Mode 下拉框被窗口裁掉。
    setMinimumSize(1420, 1020);
    resize(qMax(width(), 1420), qMax(height(), 1020));

    QLabel *colorPreviewTextLabel =
        findLabelContainsText(ui->centralwidget, QStringList() << "Color Preview");

    if (colorPreviewTextLabel != nullptr) {
        colorPreviewTextLabel->setText("");
        colorPreviewTextLabel->hide();
        colorPreviewTextLabel->setEnabled(false);
    }

    // 改颜色

    ui->centralwidget->setStyleSheet(
        "QWidget#centralwidget {"
        " background-color: #f0f4f8;"
        "}"
        );

    // 统一绿色按钮：Connect / LED toggle / Save / Reset Angle Zero
    ui->btnConnect->setStyleSheet(GREEN_BUTTON_STYLE);
    ui->btnLedOn->setText(ledOn ? "LED OFF" : "LED ON");
    ui->btnLedOn->setStyleSheet(GREEN_BUTTON_STYLE);

    // 合并 LED ON / LED OFF：只保留 btnLedOn 作为单个切换按钮。
    // btnLedOff 不再参与显示、点击和布局计算。
    ui->btnLedOff->hide();
    ui->btnLedOff->setEnabled(false);

    gLedToggleButton = ui->btnLedOn;
    gLedSliderR = ui->sliderR;
    gLedSliderG = ui->sliderG;
    gLedSliderB = ui->sliderB;
    gLedColorPreview = ui->colorPreview;

    moveConnectionBlockUp(ui, 30);   // 数字越大，整体越往上
    setupConnectionFrame(ui);        // 重新计算外框位置

    // 删除右上角 Button Help / 注释信息框，只在日志白框里显示 Wi-Fi 连接提示
    gCommTextLabel = ui->centralwidget->findChild<QLabel *>("CommTextLabel");
    if (gCommTextLabel != nullptr) {
        gCommTextLabel->hide();
        gCommTextLabel->setEnabled(false);
    }

    ui->textBrowserLog->setPlainText(
        "Please connect to \"ESP32_Motor\" Wi-Fi, Pass: \"12345678\", then click Connect."
        );

    // Status Log / LED / 参数区的位置会在默认参数控件创建后统一排版。

    // 确保 Save Parameter 按钮一定连接到保存函数。
    // Qt Designer 自动连接依赖 objectName；这里再显式连接一次，避免按钮 objectName/自动连接异常导致点击无效。
    connect(ui->btnSaveParameter, &QPushButton::clicked,
            this, &MainWindow::on_btnSaveParameter_clicked,
            Qt::UniqueConnection);

    socket = new QTcpSocket(this);

    // ESTOP 后 LED 自动红色闪烁：红 -> 灭 -> 红 -> 灭，直到 UNLOCK ESTOP。
    gEstopLedFlashTimer = new QTimer(this);
    gEstopLedFlashTimer->setInterval(500);

    gEstopLedFlashStep = [=]() {
        if (!gEstopActive || socket->state() != QAbstractSocket::ConnectedState) {
            return;
        }

        if (gEstopLedFlashRedPhase) {
            sendCommand("LED_ON", false);
            if (gLastCommandWriteOk) {
                sendCommand("RGB:255,0,0", false);
            }

            if (ui->colorPreview != nullptr) {
                ui->colorPreview->setStyleSheet(
                    "background-color: rgb(255,0,0);"
                    "border: 2px solid gray;"
                    );
            }
        } else {
            sendCommand("LED_OFF", false);

            if (ui->colorPreview != nullptr) {
                ui->colorPreview->setStyleSheet(
                    "background-color: black;"
                    "border: 2px solid gray;"
                    );
            }
        }

        gEstopLedFlashRedPhase = !gEstopLedFlashRedPhase;
    };

    gEstopLedFlashStop = [=]() {
        if (gEstopLedFlashTimer != nullptr) {
            gEstopLedFlashTimer->stop();
        }

        gEstopLedFlashRedPhase = true;

        if (socket->state() == QAbstractSocket::ConnectedState) {
            sendCommand("LED_OFF", false);
        }

        ledOn = false;
        ui->btnLedOn->setText("LED ON");
        ui->btnLedOn->setStyleSheet(GREEN_BUTTON_STYLE);

        if (ui->colorPreview != nullptr) {
            ui->colorPreview->setStyleSheet(
                "background-color: black;"
                "border: 2px solid gray;"
                );
        }
    };

    connect(gEstopLedFlashTimer, &QTimer::timeout, this, [=]() {
        if (gEstopLedFlashStep) {
            gEstopLedFlashStep();
        }
    });

    // =====================================================
    // Default IP and port
    // =====================================================
    ui->lineEditIP->setText("192.168.4.1");
    ui->lineEditPort->setText("8080");

    ui->labelConnectStatus->setText("Disconnected");
    ui->labelConnectStatus->setStyleSheet("color:red;");

    // =====================================================
    // Default parameter display
    // PID + single Speed parameter
    // =====================================================
    ui->kpEdit->setText("5.0");
    ui->kiEdit->setText("0.015");
    ui->kdEdit->setText("0.0");

    ui->speedEdit->setText("500");
    setupAbsAngleLimitEditor(ui);
    setupSpeedUnitLabel(ui);

    // 输入框列整体右移，给左侧标签（尤其 Current Angle）留完整显示空间。
    shiftInputColumnsRight(ui);

    // CPR 和 Tolerance 由 ESP32 固件固定/默认处理，上位机不再显示和保存这两个参数。
    hideParameterEditor(findChild<QWidget *>("CPREdit"), QStringList() << "CPR" << "Encoder CPR" << "Encoder");
    hideParameterEditor(findChild<QWidget *>("toleranceEdit"), QStringList() << "Tolerance" << "TOL");

    // 新布局：
    // 1) Speed / Abs Angle Limit 放到 KD 下方，并用参数框圈住；
    // 2) Status Log 区域左对齐 Wi-Fi 模块；
    // 3) LED 控制区放到右侧，也就是 ESTOP 上方；
    // 4) Motor Angle / Current Angle 整体下移，避免和 Abs Angle Limit 重叠。
    arrangeSpeedAndAngleLimitUnderKd(ui);
    positionParameterBlockUpAndSaveButton(ui);
    setupParameterFrame(ui);

    ui->currentAngleEdit->setReadOnly(true);

    // Angle send mode combo box + initial button state
    // 这里会创建 ESTOP / Restart / Reset Angle Zero，所以 LED 和 Motor Angle 的最终位置放在这之后调整。
    // Angle send mode combo box + initial button state
    setupAngleModeCombo(ui);
    setupParameterLabelFont(ui);

    // 把 ESTOP 和 Restart ESP32 放到右上角
    positionEstopRestartTopRight(ui);

    // 最终排版顺序：
    // 1) 先放好下方 Send Angle 整体区域并生成外框；
    // 2) LED 右边框和 Status Log 右边框都以这个外框右边为准。
    positionMotorAngleBlockBelowParameter(ui);
    positionSendAngleBlockBottomRight(ui);
    setupAngleControlFrame(ui);
    positionLedControlAboveEstop(ui);
    setupLedControlFrame(ui);
    setupMiddleImageInterface(ui);
    positionMiddleImageBetweenParameterAndLed(ui);
    positionStatusLogLeftAlignedWithWifi(ui);


    if (gOpenUserFileButton != nullptr) {
        connect(gOpenUserFileButton, &QPushButton::clicked, this, [=]() {
            // user.txt 放在程序 exe 同一个文件夹里。
            const QString userFilePath = QCoreApplication::applicationDirPath() + "/user.txt";
            const QFileInfo userFileInfo(userFilePath);

            if (!userFileInfo.exists() || !userFileInfo.isFile()) {
                setSendAngleStatusFailed();
                addLog("user.txt not found: " + userFileInfo.absoluteFilePath());
                return;
            }

            const bool opened = QDesktopServices::openUrl(
                QUrl::fromLocalFile(userFileInfo.absoluteFilePath())
                );

            if (opened) {
                addLog("Opened user.txt: " + userFileInfo.absoluteFilePath());
            } else {
                setSendAngleStatusFailed();
                addLog("Failed to open user.txt: " + userFileInfo.absoluteFilePath());
            }
        });
    }

    if (gEstopButton != nullptr) {
        connect(gEstopButton, &QPushButton::clicked, this, [=]() {
            if (socket->state() != QAbstractSocket::ConnectedState) {
                setSendAngleStatusFailed();
                addLog(gEstopActive ? "Not connected, ESTOP_CLEAR command not sent"
                                    : "Not connected, ESTOP command not sent");
                return;
            }

            if (gEstopActive) {
                // 解锁急停：让 ESP32 清除 latch 并重新使能 A4988。
                sendCommand("ESTOP_CLEAR", true);

                if (gLastCommandWriteOk) {
                    addLog("Emergency stop clear command sent");
                } else {
                    addLog("Emergency stop clear send failed");
                }

                return;
            }

            // 上位机立即停止 back-and-forth 的后续发送；真正断使能由 ESP32 收到 ESTOP 后执行。
            gMotorMoving = false;
            gAngleSequenceRunning = false;
            gAngleSequenceStopRequested = false;
            gBackAndForthBaseAngle = 0.0;
            gBackAndForthSentCount = 0;
            setSendAngleIdle(ui);
            setSendAngleStatusFailed();

            sendCommand("ESTOP", true);

            if (gLastCommandWriteOk) {
                setEstopButtonState(true);
                addLog("Emergency stop command sent. LED control locked and red flashing started.");
            } else {
                addLog("Emergency stop send failed");
            }
        });
    }

    if (gRestartEsp32Button != nullptr) {
        connect(gRestartEsp32Button, &QPushButton::clicked, this, [=]() {
            if (socket->state() != QAbstractSocket::ConnectedState) {
                setSendAngleStatusFailed();
                addLog("Not connected, restart command not sent");
                return;
            }

            // 重启前停止上位机继续发 back-and-forth，真正重启由 ESP32 执行。
            gMotorMoving = false;
            gAngleSequenceRunning = false;
            gAngleSequenceStopRequested = false;
            gBackAndForthBaseAngle = 0.0;
            gBackAndForthSentCount = 0;
            setSendAngleIdle(ui);
            setSendAngleStatusIdle("Restarting");

            // 当前 ESP32 固件里的重启命令是 ESTOP_REBOOT：先急停、清队列，然后重启。
            // 重启后 ESP32 会保持 ESTOP latch，需要重新连接后按 UNLOCK ESTOP。
            sendCommand("ESTOP_REBOOT", true);

            if (gLastCommandWriteOk) {
                setEstopButtonState(true);
                addLog("Restart ESP32 command sent. After reconnect, press UNLOCK ESTOP if it is latched.");
            } else {
                setSendAngleStatusFailed();
                addLog("Restart ESP32 send failed");
            }
        });
    }

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

    if (gAbsAngleLimitEdit != nullptr) {
        gAbsAngleLimitEdit->setValidator(
            new QDoubleValidator(0.0, 64800.0, 2, this)
            );
    }


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
        ui->labelConnectStatus->setStyleSheet("color:#2ecc71;");
        ui->btnConnect->setText("Disconnect");
        ui->btnConnect->setStyleSheet(GREEN_BUTTON_STYLE);

        addLog("Connected to ESP32");

        // Read current ESP32 config automatically after connection
        QTimer::singleShot(200, this, [=]() {
            sendCommand("CFG?", true);
            sendCommand("ESTOP?", true);
        });

        angleTimer->start();
    });

    connect(socket, &QTcpSocket::disconnected, this, [=]() {
        ui->labelConnectStatus->setText("Disconnected");
        ui->labelConnectStatus->setStyleSheet("color:red;");
        ui->btnConnect->setText("Connect");
        ui->btnConnect->setStyleSheet(GREEN_BUTTON_STYLE);

        addLog("Disconnected");
        if (gMotorMoving || gAngleSequenceRunning) {
            setSendAngleStatusFailed();
        }

        // Clear motor busy state on disconnect to avoid a permanently locked button
        gMotorMoving = false;
        gAngleSequenceRunning = false;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = 0.0;
        gBackAndForthSentCount = 0;
        setSendAngleIdle(ui);
        setEstopButtonState(false);

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

    applyModeFontToWholeUi(ui);

    // Enable whole-window linear scaling after all dynamic widgets have been created.
    setupResponsiveScaling(this, ui);
}

MainWindow::~MainWindow()
{
    gEstopLedFlashStep = nullptr;
    gEstopLedFlashStop = nullptr;
    gEstopLedFlashTimer = nullptr;
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
    gLastCommandWriteOk = false;

    if (socket->state() != QAbstractSocket::ConnectedState) {
        addLog("Not connected");
        return;
    }

    QString line = cmd.trimmed() + "\n";
    QByteArray data = line.toUtf8();
    qint64 written = socket->write(data);

    if (written != data.size()) {
        addLog("Socket write warning: " + socket->errorString());
    } else {
        gLastCommandWriteOk = true;
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

    v = valueFromLine(line, "ANGLE_LIMIT");
    if (!v.isEmpty() && gAbsAngleLimitEdit != nullptr) {
        gAbsAngleLimitEdit->setText(v);
    }

}

void MainWindow::handleEsp32Line(const QString &line)
{
    // Save Parameter 状态机：
    // step 0: 已发 PID，等待 OK PID
    // step 1: 已发 SPEED，等待 OK SPEED
    // step 2: 已发 ANGLE_LIMIT，等待 OK ANGLE_LIMIT
    // step 3: 已发 CFG_SAVE，等待 OK CFG_SAVE
    // step 4: 已发 CFG?，等待 CFG 回读
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
        } else if (gSaveParameterStep == 2 && line.startsWith("OK ANGLE_LIMIT")) {
            updateConfigEditsFromLine(line);
            advanceSaveStep = true;
        } else if (gSaveParameterStep == 3 && line.startsWith("OK CFG_SAVE")) {
            updateConfigEditsFromLine(line);
            addLog("ESP32 confirmed: PID, Speed and Angle Limit written to Flash");
            advanceSaveStep = true;
        } else if (gSaveParameterStep == 4 && line.startsWith("CFG ")) {
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

    // ESP32 emergency stop reply.
    // 注意：OK ESTOP_CLEAR 也以 "OK ESTOP" 开头，所以必须先判断 CLEAR。
    if (line.startsWith("OK ESTOP_CLEAR") ||
        line.startsWith("OK STOP_CLEAR") ||
        line.startsWith("OK MOTOR_ENABLE") ||
        line.startsWith("ESTOP=OFF")) {
        setEstopButtonState(false);
        setSendAngleStatusIdle("ESTOP Cleared");
        addLog("Emergency stop cleared");
        return;
    }

    if (line.startsWith("OK ESTOP ") ||
        line.startsWith("OK ESTOP_REBOOT") ||
        line.startsWith("OK STOP ") ||
        line.startsWith("OK EMERGENCY_STOP") ||
        line.startsWith("OK MOTOR_DISABLE") ||
        line.startsWith("ESTOP=ON")) {
        gMotorMoving = false;
        gAngleSequenceRunning = false;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = 0.0;
        gBackAndForthSentCount = 0;
        setSendAngleIdle(ui);
        setSendAngleStatusFailed();
        setEstopButtonState(true);
        addLog("Emergency stop active");
        return;
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

        QRegularExpression statusRe("\\bSTATUS=([^\\s]+)");
        QRegularExpressionMatch statusMatch = statusRe.match(line);
        QString motorStatus = statusMatch.hasMatch() ? statusMatch.captured(1) : QString();

        if (motorStatus.isEmpty() || motorStatus == "OK" || motorStatus == "POST_CORRECT_OK") {
            setSendAngleStatusSuccess();
        } else {
            setSendAngleStatusFailed();
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
        setSendAngleStatusFailed();
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
        addLog("ESP32 confirmed: PID, Speed and Angle Limit saved to Flash");
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

    // ANGLE_LIMIT=... / OK ANGLE_LIMIT=...
    if (line.startsWith("ANGLE_LIMIT") || line.startsWith("OK ANGLE_LIMIT")) {
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
    if (gEstopActive) {
        setSendAngleStatusFailed();
        addLog("ESTOP is active. LED control is locked until UNLOCK ESTOP.");
        return;
    }

    // LED ON / LED OFF merged into one green toggle button.
    // Same behavior style as Connect:
    //   LED is OFF -> click sends LED_ON and button becomes "LED OFF"
    //   LED is ON  -> click sends LED_OFF and button becomes "LED ON"
    const bool turnOn = !ledOn;
    sendCommand(turnOn ? "LED_ON" : "LED_OFF");

    // sendCommand() keeps gLastCommandWriteOk=false when disconnected or write failed.
    // In that case, keep the old LED state and old button text.
    if (!gLastCommandWriteOk) {
        return;
    }

    ledOn = turnOn;
    ui->btnLedOn->setText(ledOn ? "LED OFF" : "LED ON");
    ui->btnLedOn->setStyleSheet(GREEN_BUTTON_STYLE);

    if (!ledOn) {
        ui->colorPreview->setStyleSheet(
            "background-color: black;"
            "border: 2px solid gray;"
            );
    } else {
        sendRGB();
    }
}

void MainWindow::on_btnLedOff_clicked()
{
    // The old OFF button is hidden. Keep this slot safe in case it is triggered manually.
    if (ledOn) {
        on_btnLedOn_clicked();
    }
}

void MainWindow::sendRGB()
{
    if (gEstopActive) {
        return;
    }

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
        setSendAngleStatusFailed();
        addLog("Motor is moving, wait for MOTOR_DONE before sending next angle");
        return;
    }

    if (socket->state() != QAbstractSocket::ConnectedState) {
        setSendAngleStatusFailed();
        addLog("Not connected");
        return;
    }

    if (gEstopActive) {
        setSendAngleStatusFailed();
        addLog("ESTOP is active. Press UNLOCK ESTOP before sending angle commands.");
        return;
    }

    QString angleText = ui->angleEdit->text().trimmed();

    if (angleText.isEmpty()) {
        setSendAngleStatusFailed();
        addLog("Angle input empty");
        return;
    }

    bool ok = false;
    double angleValue = angleText.toDouble(&ok);

    if (!ok) {
        setSendAngleStatusFailed();
        addLog("Angle format error");
        return;
    }

    if (currentAngleSendMode() == MODE_BACK_AND_FORTH && angleValue == 0.0) {
        setSendAngleStatusFailed();
        addLog("Back-and-forth angle cannot be 0");
        return;
    }

    double firstAngle = angleValue;

    if (currentAngleSendMode() == MODE_BACK_AND_FORTH) {
        // Back-and-forth mode around the current/start angle.
        // If current is C and user inputs A:
        //   first send +A to C + A, then send -2A to C - A,
        //   then +2A to C + A, and keep alternating.
        // Each segment waits for ESP32 MOTOR_DONE before the next segment is sent.
        gAngleSequenceRunning = true;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = angleValue;
        gBackAndForthSentCount = 1;
        firstAngle = angleValue;

        addLog(
            QString("Back-and-forth infinite start: first=%1, then alternate %2 / %3 around start position")
                .arg(formatAngleForCommand(firstAngle))
                .arg(formatAngleForCommand(-2.0 * angleValue))
                .arg(formatAngleForCommand(2.0 * angleValue))
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

    if (gLastCommandWriteOk) {
        setSendAngleStatusSuccess();
    } else {
        // TCP 写入失败时没有 MOTOR_DONE 回来，所以立即解锁按钮。
        setSendAngleStatusFailed();
        gMotorMoving = false;
        gAngleSequenceRunning = false;
        gAngleSequenceStopRequested = false;
        gBackAndForthBaseAngle = 0.0;
        gBackAndForthSentCount = 0;
        setSendAngleIdle(ui);
    }
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
    QString angleLimit = gAbsAngleLimitEdit != nullptr ? gAbsAngleLimitEdit->text().trimmed() : QString("60");

    if (kp.isEmpty() || ki.isEmpty() || kd.isEmpty() || speed.isEmpty() || angleLimit.isEmpty()) {
        addLog("Parameter input empty");
        return;
    }

    bool okKp = false;
    bool okKi = false;
    bool okKd = false;
    bool okSpeed = false;
    bool okAngleLimit = false;

    double kpVal = kp.toDouble(&okKp);
    double kiVal = ki.toDouble(&okKi);
    double kdVal = kd.toDouble(&okKd);
    double speedVal = speed.toDouble(&okSpeed);
    double angleLimitVal = angleLimit.toDouble(&okAngleLimit);

    if (!okKp || !okKi || !okKd || !okSpeed || !okAngleLimit) {
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

    if (angleLimitVal < 0 || angleLimitVal > 64800) {
        addLog("Abs Angle Limit range error. Use 0 to disable, or 0 - 64800 degrees.");
        return;
    }

    gSaveParameterCommands.clear();
    gSaveParameterCommands << QString("PID:%1,%2,%3").arg(kp).arg(ki).arg(kd)
                           << QString("SPEED:%1").arg(speed)
                           << QString("ANGLE_LIMIT:%1").arg(angleLimit)
                           << QString("CFG_SAVE")
                           << QString("CFG?");

    gSaveParameterRunning = true;
    gSaveParameterStep = 0;
    gSaveParameterSequenceId++;
    ui->btnSaveParameter->setEnabled(false);

    addLog("Save started: PID -> SPEED -> ANGLE_LIMIT -> CFG_SAVE -> CFG?");
    sendCommand(gSaveParameterCommands.at(0), true);

    const int sequenceId = gSaveParameterSequenceId;
    QTimer::singleShot(6000, this, [=]() {
        if (gSaveParameterRunning && sequenceId == gSaveParameterSequenceId) {
            gSaveParameterRunning = false;
            gSaveParameterCommands.clear();
            gSaveParameterStep = 0;
            ui->btnSaveParameter->setEnabled(true);
            addLog("Save timeout: did not receive expected ESP32 reply. Check ESP32 log for OK PID / OK SPEED / OK ANGLE_LIMIT / OK CFG_SAVE / CFG");
        }
    });
}

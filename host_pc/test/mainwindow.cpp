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
#include <QLineEdit>
#include <QFormLayout>
#include <QByteArray>
#include <QStringList>

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
QLineEdit *gAbsAngleLimitEdit = nullptr;
QLabel *gAbsAngleLimitLabel = nullptr;
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

const char *ESTOP_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #e53935;"
    " color: white;"
    " font-weight: bold;"
    " font-size: 20px;"
    " border-radius: 6px;"
    " padding: 8px 16px;"
    "}"
    "QPushButton:hover {"
    " background-color: #c62828;"
    "}"
    "QPushButton:pressed {"
    " background-color: #b71c1c;"
    "}";

const char *ESTOP_UNLOCK_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #43a047;"
    " color: white;"
    " font-weight: bold;"
    " font-size: 18px;"
    " border-radius: 6px;"
    " padding: 8px 16px;"
    "}"
    "QPushButton:hover {"
    " background-color: #2e7d32;"
    "}"
    "QPushButton:pressed {"
    " background-color: #1b5e20;"
    "}";

const char *RESTART_ESP32_BUTTON_STYLE =
    "QPushButton {"
    " background-color: #fb8c00;"
    " color: white;"
    " font-weight: bold;"
    " font-size: 16px;"
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
    " font-weight: bold;"
    " font-size: 24px;"
    "}";

const char *SEND_ANGLE_STATUS_SUCCESS_STYLE =
    "QLabel {"
    " color: #1e88e5;"
    " font-weight: bold;"
    " font-size: 32px;"
    "}";

const char *SEND_ANGLE_STATUS_FAILED_STYLE =
    "QLabel {"
    " color: #e74c3c;"
    " font-weight: bold;"
    " font-size: 32px;"
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

void setEstopButtonState(bool active)
{
    gEstopActive = active;

    if (gEstopButton == nullptr) {
        return;
    }

    if (active) {
        gEstopButton->setText("UNLOCK ESTOP");
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
        gAbsAngleLimitLabel = new QLabel("Abs Angle Limit:", parent);
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
        gAngleModeCombo->addItem("Back-and-Forth: +A then ±2A");

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
    setupAbsAngleLimitEditor(ui);

    // CPR 和 Tolerance 由 ESP32 固件固定/默认处理，上位机不再显示和保存这两个参数。
    hideParameterEditor(findChild<QWidget *>("CPREdit"), QStringList() << "CPR" << "Encoder CPR" << "Encoder");
    hideParameterEditor(findChild<QWidget *>("toleranceEdit"), QStringList() << "Tolerance" << "TOL");

    ui->currentAngleEdit->setReadOnly(true);

    // Angle send mode combo box + initial button state
    setupAngleModeCombo(ui);

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
                addLog("Emergency stop command sent");
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
        ui->labelConnectStatus->setStyleSheet("color:green; font-weight:bold;");
        ui->btnConnect->setText("Disconnect");

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
        ui->labelConnectStatus->setStyleSheet("color:red; font-weight:bold;");
        ui->btnConnect->setText("Connect");

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

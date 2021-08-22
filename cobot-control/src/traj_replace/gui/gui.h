/********************************************************************************
** Form generated from reading UI file 'sim_cc.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef GUI_H
#define GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_mainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *exeBtn;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QRadioButton *homeRbtn;
    QRadioButton *startPoseRbtn;
    QRadioButton *replaceTrajRbtn;
    QRadioButton *scaleVelRBtn;
    QWidget *formLayoutWidget;
    QFormLayout *formLayout;
    QLineEdit *scaleFactorTf;
    QLabel *scaleFactorLb;

    void setupUi(QMainWindow *mainWindow)
    {
        if (mainWindow->objectName().isEmpty())
            mainWindow->setObjectName(QString::fromUtf8("mainWindow"));
        mainWindow->resize(368, 166);
        centralWidget = new QWidget(mainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        exeBtn = new QPushButton(centralWidget);
        exeBtn->setObjectName(QString::fromUtf8("exeBtn"));
        exeBtn->setGeometry(QRect(10, 130, 83, 25));
        exeBtn->setCursor(QCursor(Qt::PointingHandCursor));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 10, 160, 112));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        homeRbtn = new QRadioButton(verticalLayoutWidget);
        homeRbtn->setObjectName(QString::fromUtf8("homeRbtn"));
        homeRbtn->setCursor(QCursor(Qt::PointingHandCursor));

        verticalLayout->addWidget(homeRbtn);

        startPoseRbtn = new QRadioButton(verticalLayoutWidget);
        startPoseRbtn->setObjectName(QString::fromUtf8("startPoseRbtn"));
        startPoseRbtn->setCursor(QCursor(Qt::PointingHandCursor));

        verticalLayout->addWidget(startPoseRbtn);

        replaceTrajRbtn = new QRadioButton(verticalLayoutWidget);
        replaceTrajRbtn->setObjectName(QString::fromUtf8("replaceTrajRbtn"));
        replaceTrajRbtn->setCursor(QCursor(Qt::PointingHandCursor));

        verticalLayout->addWidget(replaceTrajRbtn);

        scaleVelRBtn = new QRadioButton(verticalLayoutWidget);
        scaleVelRBtn->setObjectName(QString::fromUtf8("scaleVelRBtn"));
        scaleVelRBtn->setCursor(QCursor(Qt::PointingHandCursor));

        verticalLayout->addWidget(scaleVelRBtn);

        formLayoutWidget = new QWidget(centralWidget);
        formLayoutWidget->setObjectName(QString::fromUtf8("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(190, 100, 160, 21));
        formLayout = new QFormLayout(formLayoutWidget);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        scaleFactorTf = new QLineEdit(formLayoutWidget);
        scaleFactorTf->setObjectName(QString::fromUtf8("scaleFactorTf"));

        formLayout->setWidget(0, QFormLayout::FieldRole, scaleFactorTf);

        scaleFactorLb = new QLabel(formLayoutWidget);
        scaleFactorLb->setObjectName(QString::fromUtf8("scaleFactorLb"));

        formLayout->setWidget(0, QFormLayout::LabelRole, scaleFactorLb);

        mainWindow->setCentralWidget(centralWidget);

        retranslateUi(mainWindow);

        QMetaObject::connectSlotsByName(mainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *mainWindow)
    {
        mainWindow->setWindowTitle(QApplication::translate("mainWindow", "Trajectory Replacement", nullptr));
        exeBtn->setText(QApplication::translate("mainWindow", "Execute", nullptr));
        homeRbtn->setText(QApplication::translate("mainWindow", "Home", nullptr));
        startPoseRbtn->setText(QApplication::translate("mainWindow", "Start pose", nullptr));
        replaceTrajRbtn->setText(QApplication::translate("mainWindow", "Replace trajectory", nullptr));
        scaleVelRBtn->setText(QApplication::translate("mainWindow", "Scale velocity", nullptr));
        scaleFactorLb->setText(QApplication::translate("mainWindow", "Scale factor", nullptr));
    } // retranslateUi

};

namespace Ui {
    class mainWindow: public Ui_mainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // GUI_H

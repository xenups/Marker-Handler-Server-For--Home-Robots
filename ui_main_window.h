/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created: Sat Nov 2 12:30:51 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTableWidget>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *tab_4;
    QTableWidget *add_tableWidget;
    QTableWidget *dataBase_tableWidget;
    QPushButton *delete_pushButton;
    QCheckBox *fillFromTf_checkBox;
    QPushButton *update_pushButton;
    QPushButton *add_pushButton;
    QLabel *dataBaselabel;
    QLabel *dataBaselabel_2;
    QCheckBox *selectAll_checkBox;
    QPushButton *goto_pushButton;
    QMenuBar *menuBar;
    QMenu *menuFarshid;
    QMenu *menuAbout;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->setEnabled(true);
        MainWindowDesign->resize(948, 543);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/Dashboard.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setTabsClosable(false);
        tabWidget->setMovable(true);
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        add_tableWidget = new QTableWidget(tab_4);
        if (add_tableWidget->columnCount() < 8)
            add_tableWidget->setColumnCount(8);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(6, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        add_tableWidget->setHorizontalHeaderItem(7, __qtablewidgetitem7);
        if (add_tableWidget->rowCount() < 1)
            add_tableWidget->setRowCount(1);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        add_tableWidget->setVerticalHeaderItem(0, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 0, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 1, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 2, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 3, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 4, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 5, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 6, __qtablewidgetitem15);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        add_tableWidget->setItem(0, 7, __qtablewidgetitem16);
        add_tableWidget->setObjectName(QString::fromUtf8("add_tableWidget"));
        add_tableWidget->setGeometry(QRect(0, 350, 821, 81));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(add_tableWidget->sizePolicy().hasHeightForWidth());
        add_tableWidget->setSizePolicy(sizePolicy);
        add_tableWidget->setMinimumSize(QSize(300, 0));
        add_tableWidget->setMaximumSize(QSize(16777215, 300));
        QPalette palette;
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(85, 170, 127, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        QBrush brush2(QColor(127, 255, 191, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush2);
        QBrush brush3(QColor(106, 212, 159, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        QBrush brush4(QColor(42, 85, 63, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush4);
        QBrush brush5(QColor(56, 113, 84, 255));
        brush5.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        QBrush brush6(QColor(255, 255, 255, 255));
        brush6.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush);
        QBrush brush7(QColor(170, 212, 191, 255));
        brush7.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush7);
        QBrush brush8(QColor(255, 255, 220, 255));
        brush8.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush7);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush8);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        add_tableWidget->setPalette(palette);
        add_tableWidget->setMouseTracking(true);
        add_tableWidget->setFocusPolicy(Qt::TabFocus);
        add_tableWidget->setFrameShadow(QFrame::Raised);
        add_tableWidget->setGridStyle(Qt::SolidLine);
        dataBase_tableWidget = new QTableWidget(tab_4);
        if (dataBase_tableWidget->columnCount() < 9)
            dataBase_tableWidget->setColumnCount(9);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        __qtablewidgetitem17->setText(QString::fromUtf8("Name"));
        __qtablewidgetitem17->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        dataBase_tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem18);
        QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(2, __qtablewidgetitem19);
        QTableWidgetItem *__qtablewidgetitem20 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(3, __qtablewidgetitem20);
        QTableWidgetItem *__qtablewidgetitem21 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(4, __qtablewidgetitem21);
        QTableWidgetItem *__qtablewidgetitem22 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(5, __qtablewidgetitem22);
        QTableWidgetItem *__qtablewidgetitem23 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(6, __qtablewidgetitem23);
        QTableWidgetItem *__qtablewidgetitem24 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(7, __qtablewidgetitem24);
        QTableWidgetItem *__qtablewidgetitem25 = new QTableWidgetItem();
        dataBase_tableWidget->setHorizontalHeaderItem(8, __qtablewidgetitem25);
        if (dataBase_tableWidget->rowCount() < 1)
            dataBase_tableWidget->setRowCount(1);
        QTableWidgetItem *__qtablewidgetitem26 = new QTableWidgetItem();
        dataBase_tableWidget->setVerticalHeaderItem(0, __qtablewidgetitem26);
        QTableWidgetItem *__qtablewidgetitem27 = new QTableWidgetItem();
        __qtablewidgetitem27->setTextAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        dataBase_tableWidget->setItem(0, 0, __qtablewidgetitem27);
        dataBase_tableWidget->setObjectName(QString::fromUtf8("dataBase_tableWidget"));
        dataBase_tableWidget->setGeometry(QRect(0, 50, 820, 271));
        dataBase_tableWidget->setMaximumSize(QSize(820, 271));
        QPalette palette1;
        palette1.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette1.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette1.setBrush(QPalette::Active, QPalette::Light, brush2);
        palette1.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        palette1.setBrush(QPalette::Active, QPalette::Dark, brush4);
        palette1.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette1.setBrush(QPalette::Active, QPalette::Text, brush);
        palette1.setBrush(QPalette::Active, QPalette::BrightText, brush6);
        palette1.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette1.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette1.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette1.setBrush(QPalette::Active, QPalette::Shadow, brush);
        palette1.setBrush(QPalette::Active, QPalette::AlternateBase, brush7);
        palette1.setBrush(QPalette::Active, QPalette::ToolTipBase, brush8);
        palette1.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette1.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette1.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette1.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette1.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette1.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::BrightText, brush6);
        palette1.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette1.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette1.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush7);
        palette1.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush8);
        palette1.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette1.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette1.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette1.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette1.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette1.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette1.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette1.setBrush(QPalette::Disabled, QPalette::BrightText, brush6);
        palette1.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette1.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette1.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette1.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette1.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush8);
        palette1.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        dataBase_tableWidget->setPalette(palette1);
        dataBase_tableWidget->setFocusPolicy(Qt::StrongFocus);
        dataBase_tableWidget->setAutoFillBackground(false);
        dataBase_tableWidget->setFrameShadow(QFrame::Sunken);
        dataBase_tableWidget->setMidLineWidth(0);
        dataBase_tableWidget->setAlternatingRowColors(true);
        dataBase_tableWidget->setShowGrid(true);
        dataBase_tableWidget->setGridStyle(Qt::SolidLine);
        delete_pushButton = new QPushButton(tab_4);
        delete_pushButton->setObjectName(QString::fromUtf8("delete_pushButton"));
        delete_pushButton->setGeometry(QRect(730, 430, 85, 27));
        fillFromTf_checkBox = new QCheckBox(tab_4);
        fillFromTf_checkBox->setObjectName(QString::fromUtf8("fillFromTf_checkBox"));
        fillFromTf_checkBox->setGeometry(QRect(630, 320, 80, 22));
        update_pushButton = new QPushButton(tab_4);
        update_pushButton->setObjectName(QString::fromUtf8("update_pushButton"));
        update_pushButton->setGeometry(QRect(640, 430, 85, 27));
        add_pushButton = new QPushButton(tab_4);
        add_pushButton->setObjectName(QString::fromUtf8("add_pushButton"));
        add_pushButton->setGeometry(QRect(550, 430, 80, 27));
        add_pushButton->setDefault(true);
        dataBaselabel = new QLabel(tab_4);
        dataBaselabel->setObjectName(QString::fromUtf8("dataBaselabel"));
        dataBaselabel->setGeometry(QRect(10, 10, 91, 31));
        QPalette palette2;
        palette2.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush9(QColor(170, 170, 127, 255));
        brush9.setStyle(Qt::SolidPattern);
        palette2.setBrush(QPalette::Active, QPalette::Button, brush9);
        QBrush brush10(QColor(255, 255, 191, 255));
        brush10.setStyle(Qt::SolidPattern);
        palette2.setBrush(QPalette::Active, QPalette::Light, brush10);
        QBrush brush11(QColor(212, 212, 159, 255));
        brush11.setStyle(Qt::SolidPattern);
        palette2.setBrush(QPalette::Active, QPalette::Midlight, brush11);
        QBrush brush12(QColor(85, 85, 63, 255));
        brush12.setStyle(Qt::SolidPattern);
        palette2.setBrush(QPalette::Active, QPalette::Dark, brush12);
        QBrush brush13(QColor(113, 113, 84, 255));
        brush13.setStyle(Qt::SolidPattern);
        palette2.setBrush(QPalette::Active, QPalette::Mid, brush13);
        palette2.setBrush(QPalette::Active, QPalette::Text, brush);
        palette2.setBrush(QPalette::Active, QPalette::BrightText, brush6);
        palette2.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette2.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette2.setBrush(QPalette::Active, QPalette::Window, brush9);
        palette2.setBrush(QPalette::Active, QPalette::Shadow, brush);
        QBrush brush14(QColor(212, 212, 191, 255));
        brush14.setStyle(Qt::SolidPattern);
        palette2.setBrush(QPalette::Active, QPalette::AlternateBase, brush14);
        palette2.setBrush(QPalette::Active, QPalette::ToolTipBase, brush8);
        palette2.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::Button, brush9);
        palette2.setBrush(QPalette::Inactive, QPalette::Light, brush10);
        palette2.setBrush(QPalette::Inactive, QPalette::Midlight, brush11);
        palette2.setBrush(QPalette::Inactive, QPalette::Dark, brush12);
        palette2.setBrush(QPalette::Inactive, QPalette::Mid, brush13);
        palette2.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::BrightText, brush6);
        palette2.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette2.setBrush(QPalette::Inactive, QPalette::Window, brush9);
        palette2.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette2.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush14);
        palette2.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush8);
        palette2.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette2.setBrush(QPalette::Disabled, QPalette::WindowText, brush12);
        palette2.setBrush(QPalette::Disabled, QPalette::Button, brush9);
        palette2.setBrush(QPalette::Disabled, QPalette::Light, brush10);
        palette2.setBrush(QPalette::Disabled, QPalette::Midlight, brush11);
        palette2.setBrush(QPalette::Disabled, QPalette::Dark, brush12);
        palette2.setBrush(QPalette::Disabled, QPalette::Mid, brush13);
        palette2.setBrush(QPalette::Disabled, QPalette::Text, brush12);
        palette2.setBrush(QPalette::Disabled, QPalette::BrightText, brush6);
        palette2.setBrush(QPalette::Disabled, QPalette::ButtonText, brush12);
        palette2.setBrush(QPalette::Disabled, QPalette::Base, brush9);
        palette2.setBrush(QPalette::Disabled, QPalette::Window, brush9);
        palette2.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette2.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush9);
        palette2.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush8);
        palette2.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        dataBaselabel->setPalette(palette2);
        QFont font;
        font.setFamily(QString::fromUtf8("Umpush"));
        font.setPointSize(14);
        dataBaselabel->setFont(font);
        dataBaselabel_2 = new QLabel(tab_4);
        dataBaselabel_2->setObjectName(QString::fromUtf8("dataBaselabel_2"));
        dataBaselabel_2->setGeometry(QRect(10, 320, 61, 31));
        QPalette palette3;
        palette3.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette3.setBrush(QPalette::Active, QPalette::Button, brush9);
        palette3.setBrush(QPalette::Active, QPalette::Light, brush10);
        palette3.setBrush(QPalette::Active, QPalette::Midlight, brush11);
        palette3.setBrush(QPalette::Active, QPalette::Dark, brush12);
        palette3.setBrush(QPalette::Active, QPalette::Mid, brush13);
        palette3.setBrush(QPalette::Active, QPalette::Text, brush);
        palette3.setBrush(QPalette::Active, QPalette::BrightText, brush6);
        palette3.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette3.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette3.setBrush(QPalette::Active, QPalette::Window, brush9);
        palette3.setBrush(QPalette::Active, QPalette::Shadow, brush);
        palette3.setBrush(QPalette::Active, QPalette::AlternateBase, brush14);
        palette3.setBrush(QPalette::Active, QPalette::ToolTipBase, brush8);
        palette3.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette3.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette3.setBrush(QPalette::Inactive, QPalette::Button, brush9);
        palette3.setBrush(QPalette::Inactive, QPalette::Light, brush10);
        palette3.setBrush(QPalette::Inactive, QPalette::Midlight, brush11);
        palette3.setBrush(QPalette::Inactive, QPalette::Dark, brush12);
        palette3.setBrush(QPalette::Inactive, QPalette::Mid, brush13);
        palette3.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette3.setBrush(QPalette::Inactive, QPalette::BrightText, brush6);
        palette3.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette3.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette3.setBrush(QPalette::Inactive, QPalette::Window, brush9);
        palette3.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette3.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush14);
        palette3.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush8);
        palette3.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette3.setBrush(QPalette::Disabled, QPalette::WindowText, brush12);
        palette3.setBrush(QPalette::Disabled, QPalette::Button, brush9);
        palette3.setBrush(QPalette::Disabled, QPalette::Light, brush10);
        palette3.setBrush(QPalette::Disabled, QPalette::Midlight, brush11);
        palette3.setBrush(QPalette::Disabled, QPalette::Dark, brush12);
        palette3.setBrush(QPalette::Disabled, QPalette::Mid, brush13);
        palette3.setBrush(QPalette::Disabled, QPalette::Text, brush12);
        palette3.setBrush(QPalette::Disabled, QPalette::BrightText, brush6);
        palette3.setBrush(QPalette::Disabled, QPalette::ButtonText, brush12);
        palette3.setBrush(QPalette::Disabled, QPalette::Base, brush9);
        palette3.setBrush(QPalette::Disabled, QPalette::Window, brush9);
        palette3.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette3.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush9);
        palette3.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush8);
        palette3.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        dataBaselabel_2->setPalette(palette3);
        dataBaselabel_2->setFont(font);
        selectAll_checkBox = new QCheckBox(tab_4);
        selectAll_checkBox->setObjectName(QString::fromUtf8("selectAll_checkBox"));
        selectAll_checkBox->setGeometry(QRect(720, 320, 97, 22));
        goto_pushButton = new QPushButton(tab_4);
        goto_pushButton->setObjectName(QString::fromUtf8("goto_pushButton"));
        goto_pushButton->setGeometry(QRect(460, 430, 80, 27));
        goto_pushButton->setDefault(true);
        tabWidget->addTab(tab_4, QString());

        gridLayout->addWidget(tabWidget, 1, 0, 1, 1);

        MainWindowDesign->setCentralWidget(centralwidget);
        menuBar = new QMenuBar(MainWindowDesign);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 948, 23));
        menuFarshid = new QMenu(menuBar);
        menuFarshid->setObjectName(QString::fromUtf8("menuFarshid"));
        menuAbout = new QMenu(menuBar);
        menuAbout->setObjectName(QString::fromUtf8("menuAbout"));
        MainWindowDesign->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindowDesign);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindowDesign->setStatusBar(statusBar);

        menuBar->addAction(menuFarshid->menuAction());
        menuBar->addAction(menuAbout->menuAction());
        menuFarshid->addSeparator();

        retranslateUi(MainWindowDesign);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "Robina scenario handler", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem = add_tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("MainWindowDesign", "Name", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem1 = add_tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("MainWindowDesign", "POS.X", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem2 = add_tableWidget->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("MainWindowDesign", "POS.Y", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem3 = add_tableWidget->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("MainWindowDesign", "POS.Z", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem4 = add_tableWidget->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QApplication::translate("MainWindowDesign", "ORI.X", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem5 = add_tableWidget->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QApplication::translate("MainWindowDesign", "ORI.Y", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem6 = add_tableWidget->horizontalHeaderItem(6);
        ___qtablewidgetitem6->setText(QApplication::translate("MainWindowDesign", "ORI.Z", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem7 = add_tableWidget->horizontalHeaderItem(7);
        ___qtablewidgetitem7->setText(QApplication::translate("MainWindowDesign", "ORI.W", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem8 = add_tableWidget->verticalHeaderItem(0);
        ___qtablewidgetitem8->setText(QApplication::translate("MainWindowDesign", "1", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled = add_tableWidget->isSortingEnabled();
        add_tableWidget->setSortingEnabled(false);
        add_tableWidget->setSortingEnabled(__sortingEnabled);

#ifndef QT_NO_TOOLTIP
        add_tableWidget->setToolTip(QApplication::translate("MainWindowDesign", "<html><head/><body><p>add items here</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        add_tableWidget->setWhatsThis(QApplication::translate("MainWindowDesign", "<html><head/><body><p>you can edit data here </p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        QTableWidgetItem *___qtablewidgetitem9 = dataBase_tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem9->setText(QApplication::translate("MainWindowDesign", "POS.X", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem10 = dataBase_tableWidget->horizontalHeaderItem(2);
        ___qtablewidgetitem10->setText(QApplication::translate("MainWindowDesign", "POS.Y", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem11 = dataBase_tableWidget->horizontalHeaderItem(3);
        ___qtablewidgetitem11->setText(QApplication::translate("MainWindowDesign", "POS.Z", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem12 = dataBase_tableWidget->horizontalHeaderItem(4);
        ___qtablewidgetitem12->setText(QApplication::translate("MainWindowDesign", "ORI.X", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem13 = dataBase_tableWidget->horizontalHeaderItem(5);
        ___qtablewidgetitem13->setText(QApplication::translate("MainWindowDesign", "ORI.Y", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem14 = dataBase_tableWidget->horizontalHeaderItem(6);
        ___qtablewidgetitem14->setText(QApplication::translate("MainWindowDesign", "ORI.Z", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem15 = dataBase_tableWidget->horizontalHeaderItem(7);
        ___qtablewidgetitem15->setText(QApplication::translate("MainWindowDesign", "ORI.W", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem16 = dataBase_tableWidget->horizontalHeaderItem(8);
        ___qtablewidgetitem16->setText(QApplication::translate("MainWindowDesign", "ID", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem17 = dataBase_tableWidget->verticalHeaderItem(0);
        ___qtablewidgetitem17->setText(QApplication::translate("MainWindowDesign", "1", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled1 = dataBase_tableWidget->isSortingEnabled();
        dataBase_tableWidget->setSortingEnabled(false);
        dataBase_tableWidget->setSortingEnabled(__sortingEnabled1);

#ifndef QT_NO_TOOLTIP
        dataBase_tableWidget->setToolTip(QApplication::translate("MainWindowDesign", "<html><head/><body><p>database items</p><p>for update an item, edit item here then press update !</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_WHATSTHIS
        dataBase_tableWidget->setWhatsThis(QApplication::translate("MainWindowDesign", "<html><head/><body><p>This is database table</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
#ifndef QT_NO_TOOLTIP
        delete_pushButton->setToolTip(QApplication::translate("MainWindowDesign", "<html><head/><body><p>Press delete </p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        delete_pushButton->setText(QApplication::translate("MainWindowDesign", "Delete", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        fillFromTf_checkBox->setToolTip(QApplication::translate("MainWindowDesign", "<html><head/><body><p>Press F3 </p></body></html>", "Press F3", QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        fillFromTf_checkBox->setText(QApplication::translate("MainWindowDesign", "Manual", 0, QApplication::UnicodeUTF8));
        update_pushButton->setText(QApplication::translate("MainWindowDesign", "Update", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        add_pushButton->setToolTip(QApplication::translate("MainWindowDesign", "<html><head/><body><p>Press Enter to add</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        add_pushButton->setText(QApplication::translate("MainWindowDesign", "Add", 0, QApplication::UnicodeUTF8));
        dataBaselabel->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#00aaff;\">DataBase :</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        dataBaselabel_2->setText(QApplication::translate("MainWindowDesign", "<html><head/><body><p><span style=\" font-size:12pt; font-weight:600; color:#00aaff;\">Add :</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        selectAll_checkBox->setText(QApplication::translate("MainWindowDesign", "Select All", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        goto_pushButton->setToolTip(QApplication::translate("MainWindowDesign", "<html><head/><body><p>Press Enter to add</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        goto_pushButton->setText(QApplication::translate("MainWindowDesign", "Goto", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("MainWindowDesign", "Location", 0, QApplication::UnicodeUTF8));
        menuFarshid->setTitle(QApplication::translate("MainWindowDesign", "file", 0, QApplication::UnicodeUTF8));
        menuAbout->setTitle(QApplication::translate("MainWindowDesign", "Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H

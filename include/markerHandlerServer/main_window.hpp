#ifndef markerHandlerServer_MAIN_WINDOW_HPP
#define markerHandlerServer_MAIN_WINDOW_HPP

#include "markerHandler.hpp"

#include <QtGui/QMainWindow>
#include "ui_main_window.h"

// add marker gui
#include <QKeyEvent>
#include <QtSql/QtSql>
#include <QMessageBox>
#include <QTableWidgetItem>


class mainWindow : public QMainWindow
{
    Q_OBJECT
public:
    mainWindow(int argc, char **argv, QWidget *parent=0);
    ~mainWindow();

    //add marker//
    void update_table_dataBase();


public slots: // slots for power management


signals:
    void rosShutdown();

private slots:
    //add marker gui slots
    void addValue();
    void deleteItem();
    void goto_checked_markers();
    void updateDataBase();
    void add_tableWidget_selectAll(bool selectAll);
    void keyPressEvent(QKeyEvent *);
    bool duplicateFinder();
    void add_tableWidget_hideColumns(bool hide);
    void help();
    //log_viewer gui slots
    void onShutDownROS();
private:

    Ui::MainWindowDesign *ui;
    ros::NodeHandle *nh;
    MarkerHandler *markerHandler;
   // Navigation *navigation_client;
    markerHandlerServer::environment_markers markers;

    bool flag;
    bool visibility_gui;

    QTableWidgetItem *item ;
    dataBase_handler *db_handler;

};
#endif

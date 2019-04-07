/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Sat Nov 2 12:30:51 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_mainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      26,   11,   11,   11, 0x08,
      37,   11,   11,   11, 0x08,
      50,   11,   11,   11, 0x08,
      73,   11,   11,   11, 0x08,
     100,   90,   11,   11, 0x08,
     132,   11,   11,   11, 0x08,
     163,   11,  158,   11, 0x08,
     186,  181,   11,   11, 0x08,
     220,   11,   11,   11, 0x08,
     227,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_mainWindow[] = {
    "mainWindow\0\0rosShutdown()\0addValue()\0"
    "deleteItem()\0goto_checked_markers()\0"
    "updateDataBase()\0selectAll\0"
    "add_tableWidget_selectAll(bool)\0"
    "keyPressEvent(QKeyEvent*)\0bool\0"
    "duplicateFinder()\0hide\0"
    "add_tableWidget_hideColumns(bool)\0"
    "help()\0onShutDownROS()\0"
};

void mainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        mainWindow *_t = static_cast<mainWindow *>(_o);
        switch (_id) {
        case 0: _t->rosShutdown(); break;
        case 1: _t->addValue(); break;
        case 2: _t->deleteItem(); break;
        case 3: _t->goto_checked_markers(); break;
        case 4: _t->updateDataBase(); break;
        case 5: _t->add_tableWidget_selectAll((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->keyPressEvent((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 7: { bool _r = _t->duplicateFinder();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 8: _t->add_tableWidget_hideColumns((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->help(); break;
        case 10: _t->onShutDownROS(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData mainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject mainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_mainWindow,
      qt_meta_data_mainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &mainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *mainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *mainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_mainWindow))
        return static_cast<void*>(const_cast< mainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int mainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void mainWindow::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE

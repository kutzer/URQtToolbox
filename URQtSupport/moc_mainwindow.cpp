/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[74];
    char stringdata0[804];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 13), // "getJointSpeed"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 9), // "double[6]"
QT_MOC_LITERAL(4, 36, 6), // "target"
QT_MOC_LITERAL(5, 43, 4), // "time"
QT_MOC_LITERAL(6, 48, 13), // "sendMatlabAck"
QT_MOC_LITERAL(7, 62, 4), // "code"
QT_MOC_LITERAL(8, 67, 6), // "servoj"
QT_MOC_LITERAL(9, 74, 7), // "vector6"
QT_MOC_LITERAL(10, 82, 1), // "t"
QT_MOC_LITERAL(11, 84, 14), // "lookahead_time"
QT_MOC_LITERAL(12, 99, 4), // "gain"
QT_MOC_LITERAL(13, 104, 6), // "speedj"
QT_MOC_LITERAL(14, 111, 1), // "a"
QT_MOC_LITERAL(15, 113, 12), // "linearAdjust"
QT_MOC_LITERAL(16, 126, 4), // "axis"
QT_MOC_LITERAL(17, 131, 4), // "dist"
QT_MOC_LITERAL(18, 136, 10), // "rotateTool"
QT_MOC_LITERAL(19, 147, 3), // "rot"
QT_MOC_LITERAL(20, 151, 11), // "toolContact"
QT_MOC_LITERAL(21, 163, 5), // "stopL"
QT_MOC_LITERAL(22, 169, 4), // "aRot"
QT_MOC_LITERAL(23, 174, 5), // "stopJ"
QT_MOC_LITERAL(24, 180, 9), // "zeroForce"
QT_MOC_LITERAL(25, 190, 11), // "setWaypoint"
QT_MOC_LITERAL(26, 202, 7), // "uint8_t"
QT_MOC_LITERAL(27, 210, 11), // "std::string"
QT_MOC_LITERAL(28, 222, 3), // "arg"
QT_MOC_LITERAL(29, 226, 12), // "moveWaypoint"
QT_MOC_LITERAL(30, 239, 5), // "label"
QT_MOC_LITERAL(31, 245, 4), // "type"
QT_MOC_LITERAL(32, 250, 8), // "moveHome"
QT_MOC_LITERAL(33, 259, 9), // "storeLast"
QT_MOC_LITERAL(34, 269, 7), // "setHome"
QT_MOC_LITERAL(35, 277, 5), // "moveJ"
QT_MOC_LITERAL(36, 283, 1), // "v"
QT_MOC_LITERAL(37, 285, 1), // "r"
QT_MOC_LITERAL(38, 287, 9), // "cartesian"
QT_MOC_LITERAL(39, 297, 4), // "fast"
QT_MOC_LITERAL(40, 302, 5), // "moveL"
QT_MOC_LITERAL(41, 308, 11), // "sendCommand"
QT_MOC_LITERAL(42, 320, 7), // "command"
QT_MOC_LITERAL(43, 328, 15), // "sendCommandFast"
QT_MOC_LITERAL(44, 344, 10), // "uConnected"
QT_MOC_LITERAL(45, 355, 15), // "cycleArmConnect"
QT_MOC_LITERAL(46, 371, 13), // "uDisconnected"
QT_MOC_LITERAL(47, 385, 12), // "gReadTcpData"
QT_MOC_LITERAL(48, 398, 10), // "gConnected"
QT_MOC_LITERAL(49, 409, 13), // "gDisconnected"
QT_MOC_LITERAL(50, 423, 12), // "dReadTcpData"
QT_MOC_LITERAL(51, 436, 10), // "dConnected"
QT_MOC_LITERAL(52, 447, 13), // "dDisconnected"
QT_MOC_LITERAL(53, 461, 10), // "mConnected"
QT_MOC_LITERAL(54, 472, 13), // "mDisconnected"
QT_MOC_LITERAL(55, 486, 11), // "rtConnected"
QT_MOC_LITERAL(56, 498, 14), // "rtDisconnected"
QT_MOC_LITERAL(57, 513, 13), // "rtReadTcpData"
QT_MOC_LITERAL(58, 527, 8), // "Timeout1"
QT_MOC_LITERAL(59, 536, 8), // "Timeout2"
QT_MOC_LITERAL(60, 545, 8), // "Timeout3"
QT_MOC_LITERAL(61, 554, 22), // "on_CEdit_returnPressed"
QT_MOC_LITERAL(62, 577, 22), // "on_GEdit_returnPressed"
QT_MOC_LITERAL(63, 600, 22), // "on_DEdit_returnPressed"
QT_MOC_LITERAL(64, 623, 18), // "on_RobotPB_clicked"
QT_MOC_LITERAL(65, 642, 17), // "on_GripPB_clicked"
QT_MOC_LITERAL(66, 660, 18), // "on_pstopPB_clicked"
QT_MOC_LITERAL(67, 679, 30), // "on_actionNet_Address_triggered"
QT_MOC_LITERAL(68, 710, 16), // "acceptConnection"
QT_MOC_LITERAL(69, 727, 12), // "incomingData"
QT_MOC_LITERAL(70, 740, 11), // "socketError"
QT_MOC_LITERAL(71, 752, 28), // "QAbstractSocket::SocketError"
QT_MOC_LITERAL(72, 781, 5), // "error"
QT_MOC_LITERAL(73, 787, 16) // "socketDisconnect"

    },
    "MainWindow\0getJointSpeed\0\0double[6]\0"
    "target\0time\0sendMatlabAck\0code\0servoj\0"
    "vector6\0t\0lookahead_time\0gain\0speedj\0"
    "a\0linearAdjust\0axis\0dist\0rotateTool\0"
    "rot\0toolContact\0stopL\0aRot\0stopJ\0"
    "zeroForce\0setWaypoint\0uint8_t\0std::string\0"
    "arg\0moveWaypoint\0label\0type\0moveHome\0"
    "storeLast\0setHome\0moveJ\0v\0r\0cartesian\0"
    "fast\0moveL\0sendCommand\0command\0"
    "sendCommandFast\0uConnected\0cycleArmConnect\0"
    "uDisconnected\0gReadTcpData\0gConnected\0"
    "gDisconnected\0dReadTcpData\0dConnected\0"
    "dDisconnected\0mConnected\0mDisconnected\0"
    "rtConnected\0rtDisconnected\0rtReadTcpData\0"
    "Timeout1\0Timeout2\0Timeout3\0"
    "on_CEdit_returnPressed\0on_GEdit_returnPressed\0"
    "on_DEdit_returnPressed\0on_RobotPB_clicked\0"
    "on_GripPB_clicked\0on_pstopPB_clicked\0"
    "on_actionNet_Address_triggered\0"
    "acceptConnection\0incomingData\0socketError\0"
    "QAbstractSocket::SocketError\0error\0"
    "socketDisconnect"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      50,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,  264,    2, 0x08 /* Private */,
       6,    1,  269,    2, 0x08 /* Private */,
       8,    4,  272,    2, 0x08 /* Private */,
      13,    3,  281,    2, 0x08 /* Private */,
      15,    2,  288,    2, 0x08 /* Private */,
      18,    1,  293,    2, 0x08 /* Private */,
      20,    0,  296,    2, 0x08 /* Private */,
      21,    2,  297,    2, 0x08 /* Private */,
      21,    1,  302,    2, 0x08 /* Private */,
      23,    1,  305,    2, 0x08 /* Private */,
      24,    0,  308,    2, 0x08 /* Private */,
      25,    1,  309,    2, 0x08 /* Private */,
      29,    2,  312,    2, 0x08 /* Private */,
      32,    1,  317,    2, 0x08 /* Private */,
      33,    0,  320,    2, 0x08 /* Private */,
      34,    0,  321,    2, 0x08 /* Private */,
      35,    7,  322,    2, 0x08 /* Private */,
      35,    3,  337,    2, 0x08 /* Private */,
      40,    6,  344,    2, 0x08 /* Private */,
      40,    2,  357,    2, 0x08 /* Private */,
      41,    1,  362,    2, 0x08 /* Private */,
      43,    1,  365,    2, 0x08 /* Private */,
      44,    0,  368,    2, 0x08 /* Private */,
      45,    0,  369,    2, 0x08 /* Private */,
      46,    0,  370,    2, 0x08 /* Private */,
      47,    0,  371,    2, 0x08 /* Private */,
      48,    0,  372,    2, 0x08 /* Private */,
      49,    0,  373,    2, 0x08 /* Private */,
      50,    0,  374,    2, 0x08 /* Private */,
      51,    0,  375,    2, 0x08 /* Private */,
      52,    0,  376,    2, 0x08 /* Private */,
      53,    0,  377,    2, 0x08 /* Private */,
      54,    0,  378,    2, 0x08 /* Private */,
      55,    0,  379,    2, 0x08 /* Private */,
      56,    0,  380,    2, 0x08 /* Private */,
      57,    0,  381,    2, 0x08 /* Private */,
      58,    0,  382,    2, 0x08 /* Private */,
      59,    0,  383,    2, 0x08 /* Private */,
      60,    0,  384,    2, 0x08 /* Private */,
      61,    0,  385,    2, 0x08 /* Private */,
      62,    0,  386,    2, 0x08 /* Private */,
      63,    0,  387,    2, 0x08 /* Private */,
      64,    0,  388,    2, 0x08 /* Private */,
      65,    0,  389,    2, 0x08 /* Private */,
      66,    0,  390,    2, 0x08 /* Private */,
      67,    0,  391,    2, 0x08 /* Private */,
      68,    0,  392,    2, 0x08 /* Private */,
      69,    0,  393,    2, 0x08 /* Private */,
      70,    1,  394,    2, 0x08 /* Private */,
      73,    0,  397,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Double, 0x80000000 | 3, QMetaType::Int,    4,    5,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Double, QMetaType::Double, QMetaType::Int,    9,   10,   11,   12,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Double, QMetaType::Double,    9,   14,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::Double,   16,   17,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Bool,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   14,   22,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void,
    0x80000000 | 26, 0x80000000 | 27,   28,
    0x80000000 | 26, 0x80000000 | 27, QMetaType::Char,   30,   31,
    0x80000000 | 26, QMetaType::Char,   31,
    QMetaType::Void,
    0x80000000 | 26,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Double, QMetaType::Double, QMetaType::Int, QMetaType::Double, QMetaType::Bool, QMetaType::Bool,    9,   14,   36,   10,   37,   38,   39,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool, QMetaType::Bool,    9,   38,   39,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Double, QMetaType::Double, QMetaType::Int, QMetaType::Double, QMetaType::Bool,    9,   14,   36,   10,   37,   38,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool,    9,   38,
    QMetaType::Void, 0x80000000 | 27,   42,
    QMetaType::Void, 0x80000000 | 27,   42,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 71,   72,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: { double _r = _t->getJointSpeed((*reinterpret_cast< double(*)[6]>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 1: _t->sendMatlabAck((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->servoj((*reinterpret_cast< double(*)[6]>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 3: _t->speedj((*reinterpret_cast< double(*)[6]>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3]))); break;
        case 4: _t->linearAdjust((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 5: _t->rotateTool((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: { bool _r = _t->toolContact();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 7: _t->stopL((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 8: _t->stopL((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->stopJ((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->zeroForce(); break;
        case 11: { uint8_t _r = _t->setWaypoint((*reinterpret_cast< std::string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 12: { uint8_t _r = _t->moveWaypoint((*reinterpret_cast< std::string(*)>(_a[1])),(*reinterpret_cast< char(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 13: { uint8_t _r = _t->moveHome((*reinterpret_cast< char(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 14: _t->storeLast(); break;
        case 15: { uint8_t _r = _t->setHome();
            if (_a[0]) *reinterpret_cast< uint8_t*>(_a[0]) = std::move(_r); }  break;
        case 16: _t->moveJ((*reinterpret_cast< double(*)[6]>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6])),(*reinterpret_cast< bool(*)>(_a[7]))); break;
        case 17: _t->moveJ((*reinterpret_cast< double(*)[6]>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 18: _t->moveL((*reinterpret_cast< double(*)[6]>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6]))); break;
        case 19: _t->moveL((*reinterpret_cast< double(*)[6]>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 20: _t->sendCommand((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 21: _t->sendCommandFast((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 22: _t->uConnected(); break;
        case 23: _t->cycleArmConnect(); break;
        case 24: _t->uDisconnected(); break;
        case 25: _t->gReadTcpData(); break;
        case 26: _t->gConnected(); break;
        case 27: _t->gDisconnected(); break;
        case 28: _t->dReadTcpData(); break;
        case 29: _t->dConnected(); break;
        case 30: _t->dDisconnected(); break;
        case 31: _t->mConnected(); break;
        case 32: _t->mDisconnected(); break;
        case 33: _t->rtConnected(); break;
        case 34: _t->rtDisconnected(); break;
        case 35: _t->rtReadTcpData(); break;
        case 36: _t->Timeout1(); break;
        case 37: _t->Timeout2(); break;
        case 38: _t->Timeout3(); break;
        case 39: _t->on_CEdit_returnPressed(); break;
        case 40: _t->on_GEdit_returnPressed(); break;
        case 41: _t->on_DEdit_returnPressed(); break;
        case 42: _t->on_RobotPB_clicked(); break;
        case 43: _t->on_GripPB_clicked(); break;
        case 44: _t->on_pstopPB_clicked(); break;
        case 45: _t->on_actionNet_Address_triggered(); break;
        case 46: _t->acceptConnection(); break;
        case 47: _t->incomingData(); break;
        case 48: _t->socketError((*reinterpret_cast< QAbstractSocket::SocketError(*)>(_a[1]))); break;
        case 49: _t->socketDisconnect(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 48:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QAbstractSocket::SocketError >(); break;
            }
            break;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 50)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 50;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 50)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 50;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

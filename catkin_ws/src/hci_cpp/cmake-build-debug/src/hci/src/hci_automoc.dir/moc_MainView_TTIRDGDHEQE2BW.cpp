/****************************************************************************
** Meta object code from reading C++ file 'MainView.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/hci/src/view/MainView.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainView.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainView_t {
    QByteArrayData data[35];
    char stringdata0[453];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainView_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainView_t qt_meta_stringdata_MainView = {
    {
QT_MOC_LITERAL(0, 0, 8), // "MainView"
QT_MOC_LITERAL(1, 9, 19), // "steeringModeChanged"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 12), // "SteeringMode"
QT_MOC_LITERAL(4, 43, 4), // "mode"
QT_MOC_LITERAL(5, 48, 19), // "joystickModeChanged"
QT_MOC_LITERAL(6, 68, 8), // "modeText"
QT_MOC_LITERAL(7, 77, 20), // "closeLoopModeChanged"
QT_MOC_LITERAL(8, 98, 17), // "ArmClosedLoopMode"
QT_MOC_LITERAL(9, 116, 15), // "armJointChanged"
QT_MOC_LITERAL(10, 132, 8), // "ArmJoint"
QT_MOC_LITERAL(11, 141, 5), // "joint"
QT_MOC_LITERAL(12, 147, 14), // "armModeChanged"
QT_MOC_LITERAL(13, 162, 7), // "ArmMode"
QT_MOC_LITERAL(14, 170, 18), // "updateSteeringMode"
QT_MOC_LITERAL(15, 189, 17), // "updateDriveStatus"
QT_MOC_LITERAL(16, 207, 15), // "DriveStatusData"
QT_MOC_LITERAL(17, 223, 10), // "statusData"
QT_MOC_LITERAL(18, 234, 15), // "setInputVoltage"
QT_MOC_LITERAL(19, 250, 5), // "value"
QT_MOC_LITERAL(20, 256, 15), // "setInputCurrent"
QT_MOC_LITERAL(21, 272, 16), // "setOutputVoltage"
QT_MOC_LITERAL(22, 289, 16), // "setOutputCurrent"
QT_MOC_LITERAL(23, 306, 14), // "setOutputPower"
QT_MOC_LITERAL(24, 321, 14), // "setTemperature"
QT_MOC_LITERAL(25, 336, 14), // "setMotorEnable"
QT_MOC_LITERAL(26, 351, 6), // "enable"
QT_MOC_LITERAL(27, 358, 10), // "setArmMode"
QT_MOC_LITERAL(28, 369, 14), // "changeArmJoint"
QT_MOC_LITERAL(29, 384, 19), // "changeCloseLoopMode"
QT_MOC_LITERAL(30, 404, 8), // "setPitch"
QT_MOC_LITERAL(31, 413, 7), // "setRoll"
QT_MOC_LITERAL(32, 421, 6), // "setYaw"
QT_MOC_LITERAL(33, 428, 12), // "setLongitude"
QT_MOC_LITERAL(34, 441, 11) // "setLatitude"

    },
    "MainView\0steeringModeChanged\0\0"
    "SteeringMode\0mode\0joystickModeChanged\0"
    "modeText\0closeLoopModeChanged\0"
    "ArmClosedLoopMode\0armJointChanged\0"
    "ArmJoint\0joint\0armModeChanged\0ArmMode\0"
    "updateSteeringMode\0updateDriveStatus\0"
    "DriveStatusData\0statusData\0setInputVoltage\0"
    "value\0setInputCurrent\0setOutputVoltage\0"
    "setOutputCurrent\0setOutputPower\0"
    "setTemperature\0setMotorEnable\0enable\0"
    "setArmMode\0changeArmJoint\0changeCloseLoopMode\0"
    "setPitch\0setRoll\0setYaw\0setLongitude\0"
    "setLatitude"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainView[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      22,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  124,    2, 0x06 /* Public */,
       5,    1,  127,    2, 0x06 /* Public */,
       7,    1,  130,    2, 0x06 /* Public */,
       9,    1,  133,    2, 0x06 /* Public */,
      12,    1,  136,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      14,    1,  139,    2, 0x0a /* Public */,
      15,    1,  142,    2, 0x0a /* Public */,
      18,    1,  145,    2, 0x0a /* Public */,
      20,    1,  148,    2, 0x0a /* Public */,
      21,    1,  151,    2, 0x0a /* Public */,
      22,    1,  154,    2, 0x0a /* Public */,
      23,    1,  157,    2, 0x0a /* Public */,
      24,    1,  160,    2, 0x0a /* Public */,
      25,    1,  163,    2, 0x0a /* Public */,
      27,    1,  166,    2, 0x0a /* Public */,
      28,    1,  169,    2, 0x0a /* Public */,
      29,    1,  172,    2, 0x0a /* Public */,
      30,    1,  175,    2, 0x0a /* Public */,
      31,    1,  178,    2, 0x0a /* Public */,
      32,    1,  181,    2, 0x0a /* Public */,
      33,    1,  184,    2, 0x0a /* Public */,
      34,    1,  187,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, 0x80000000 | 8,    4,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 13,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Void, QMetaType::Double,   19,
    QMetaType::Void, QMetaType::Bool,   26,
    QMetaType::Void, 0x80000000 | 13,    4,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 8,    4,
    QMetaType::Void, QMetaType::Float,   19,
    QMetaType::Void, QMetaType::Float,   19,
    QMetaType::Void, QMetaType::Float,   19,
    QMetaType::Void, QMetaType::Float,   19,
    QMetaType::Void, QMetaType::Float,   19,

       0        // eod
};

void MainView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainView *_t = static_cast<MainView *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->steeringModeChanged((*reinterpret_cast< const SteeringMode(*)>(_a[1]))); break;
        case 1: _t->joystickModeChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->closeLoopModeChanged((*reinterpret_cast< ArmClosedLoopMode(*)>(_a[1]))); break;
        case 3: _t->armJointChanged((*reinterpret_cast< ArmJoint(*)>(_a[1]))); break;
        case 4: _t->armModeChanged((*reinterpret_cast< ArmMode(*)>(_a[1]))); break;
        case 5: _t->updateSteeringMode((*reinterpret_cast< const SteeringMode(*)>(_a[1]))); break;
        case 6: _t->updateDriveStatus((*reinterpret_cast< const DriveStatusData(*)>(_a[1]))); break;
        case 7: _t->setInputVoltage((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->setInputCurrent((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 9: _t->setOutputVoltage((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 10: _t->setOutputCurrent((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->setOutputPower((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->setTemperature((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->setMotorEnable((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: _t->setArmMode((*reinterpret_cast< ArmMode(*)>(_a[1]))); break;
        case 15: _t->changeArmJoint((*reinterpret_cast< ArmJoint(*)>(_a[1]))); break;
        case 16: _t->changeCloseLoopMode((*reinterpret_cast< ArmClosedLoopMode(*)>(_a[1]))); break;
        case 17: _t->setPitch((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 18: _t->setRoll((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 19: _t->setYaw((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 20: _t->setLongitude((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 21: _t->setLatitude((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MainView::*_t)(const SteeringMode & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainView::steeringModeChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (MainView::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainView::joystickModeChanged)) {
                *result = 1;
            }
        }
        {
            typedef void (MainView::*_t)(ArmClosedLoopMode );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainView::closeLoopModeChanged)) {
                *result = 2;
            }
        }
        {
            typedef void (MainView::*_t)(ArmJoint );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainView::armJointChanged)) {
                *result = 3;
            }
        }
        {
            typedef void (MainView::*_t)(ArmMode );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainView::armModeChanged)) {
                *result = 4;
            }
        }
    }
}

const QMetaObject MainView::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_MainView.data,
      qt_meta_data_MainView,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainView::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainView.stringdata0))
        return static_cast<void*>(const_cast< MainView*>(this));
    return QWidget::qt_metacast(_clname);
}

int MainView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 22)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 22;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 22)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 22;
    }
    return _id;
}

// SIGNAL 0
void MainView::steeringModeChanged(const SteeringMode & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainView::joystickModeChanged(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MainView::closeLoopModeChanged(ArmClosedLoopMode _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MainView::armJointChanged(ArmJoint _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MainView::armModeChanged(ArmMode _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE

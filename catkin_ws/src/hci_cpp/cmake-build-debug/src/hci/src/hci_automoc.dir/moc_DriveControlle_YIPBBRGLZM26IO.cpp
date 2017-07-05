/****************************************************************************
** Meta object code from reading C++ file 'DriveController.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/hci/src/controller/DriveController.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DriveController.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_DriveController_t {
    QByteArrayData data[17];
    char stringdata0[228];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DriveController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DriveController_t qt_meta_stringdata_DriveController = {
    {
QT_MOC_LITERAL(0, 0, 15), // "DriveController"
QT_MOC_LITERAL(1, 16, 19), // "steeringModeUpdated"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 12), // "SteeringMode"
QT_MOC_LITERAL(4, 50, 4), // "mode"
QT_MOC_LITERAL(5, 55, 18), // "motorEnableChanged"
QT_MOC_LITERAL(6, 74, 6), // "enable"
QT_MOC_LITERAL(7, 81, 18), // "wheelStatusUpdated"
QT_MOC_LITERAL(8, 100, 15), // "DriveStatusData"
QT_MOC_LITERAL(9, 116, 6), // "status"
QT_MOC_LITERAL(10, 123, 18), // "updateSteeringMode"
QT_MOC_LITERAL(11, 142, 7), // "process"
QT_MOC_LITERAL(12, 150, 12), // "enableMotors"
QT_MOC_LITERAL(13, 163, 7), // "publish"
QT_MOC_LITERAL(14, 171, 22), // "wheelStatusROSCallback"
QT_MOC_LITERAL(15, 194, 25), // "rover_common::MotorStatus"
QT_MOC_LITERAL(16, 220, 7) // "message"

    },
    "DriveController\0steeringModeUpdated\0"
    "\0SteeringMode\0mode\0motorEnableChanged\0"
    "enable\0wheelStatusUpdated\0DriveStatusData\0"
    "status\0updateSteeringMode\0process\0"
    "enableMotors\0publish\0wheelStatusROSCallback\0"
    "rover_common::MotorStatus\0message"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DriveController[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       5,    1,   57,    2, 0x06 /* Public */,
       7,    1,   60,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      10,    1,   63,    2, 0x0a /* Public */,
      11,    0,   66,    2, 0x0a /* Public */,
      12,    1,   67,    2, 0x08 /* Private */,
      13,    0,   70,    2, 0x08 /* Private */,
      14,    1,   71,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, 0x80000000 | 8,    9,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 15,   16,

       0        // eod
};

void DriveController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DriveController *_t = static_cast<DriveController *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->steeringModeUpdated((*reinterpret_cast< SteeringMode(*)>(_a[1]))); break;
        case 1: _t->motorEnableChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->wheelStatusUpdated((*reinterpret_cast< DriveStatusData(*)>(_a[1]))); break;
        case 3: _t->updateSteeringMode((*reinterpret_cast< SteeringMode(*)>(_a[1]))); break;
        case 4: _t->process(); break;
        case 5: _t->enableMotors((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->publish(); break;
        case 7: _t->wheelStatusROSCallback((*reinterpret_cast< const rover_common::MotorStatus(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DriveController::*_t)(SteeringMode );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DriveController::steeringModeUpdated)) {
                *result = 0;
            }
        }
        {
            typedef void (DriveController::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DriveController::motorEnableChanged)) {
                *result = 1;
            }
        }
        {
            typedef void (DriveController::*_t)(DriveStatusData );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DriveController::wheelStatusUpdated)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject DriveController::staticMetaObject = {
    { &JoystickInterface::staticMetaObject, qt_meta_stringdata_DriveController.data,
      qt_meta_data_DriveController,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *DriveController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DriveController::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_DriveController.stringdata0))
        return static_cast<void*>(const_cast< DriveController*>(this));
    return JoystickInterface::qt_metacast(_clname);
}

int DriveController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = JoystickInterface::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void DriveController::steeringModeUpdated(SteeringMode _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DriveController::motorEnableChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void DriveController::wheelStatusUpdated(DriveStatusData _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE

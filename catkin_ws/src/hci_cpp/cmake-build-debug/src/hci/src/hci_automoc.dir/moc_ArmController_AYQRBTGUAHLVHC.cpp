/****************************************************************************
** Meta object code from reading C++ file 'ArmController.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/hci/src/controller/ArmController.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ArmController.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ArmController_t {
    QByteArrayData data[15];
    char stringdata0[180];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ArmController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ArmController_t qt_meta_stringdata_ArmController = {
    {
QT_MOC_LITERAL(0, 0, 13), // "ArmController"
QT_MOC_LITERAL(1, 14, 18), // "motorEnableChanged"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 6), // "enable"
QT_MOC_LITERAL(4, 41, 14), // "armModeChanged"
QT_MOC_LITERAL(5, 56, 7), // "ArmMode"
QT_MOC_LITERAL(6, 64, 4), // "mode"
QT_MOC_LITERAL(7, 69, 15), // "armJointChanged"
QT_MOC_LITERAL(8, 85, 8), // "ArmJoint"
QT_MOC_LITERAL(9, 94, 5), // "joint"
QT_MOC_LITERAL(10, 100, 7), // "process"
QT_MOC_LITERAL(11, 108, 13), // "changeArmMode"
QT_MOC_LITERAL(12, 122, 19), // "changeOpenLoopJoint"
QT_MOC_LITERAL(13, 142, 19), // "changeCloseLoopMode"
QT_MOC_LITERAL(14, 162, 17) // "ArmClosedLoopMode"

    },
    "ArmController\0motorEnableChanged\0\0"
    "enable\0armModeChanged\0ArmMode\0mode\0"
    "armJointChanged\0ArmJoint\0joint\0process\0"
    "changeArmMode\0changeOpenLoopJoint\0"
    "changeCloseLoopMode\0ArmClosedLoopMode"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ArmController[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       4,    1,   52,    2, 0x06 /* Public */,
       7,    1,   55,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      10,    0,   58,    2, 0x0a /* Public */,
      11,    1,   59,    2, 0x0a /* Public */,
      12,    1,   62,    2, 0x0a /* Public */,
      13,    1,   65,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 8,    9,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, 0x80000000 | 14,    6,

       0        // eod
};

void ArmController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ArmController *_t = static_cast<ArmController *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->motorEnableChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->armModeChanged((*reinterpret_cast< ArmMode(*)>(_a[1]))); break;
        case 2: _t->armJointChanged((*reinterpret_cast< ArmJoint(*)>(_a[1]))); break;
        case 3: _t->process(); break;
        case 4: _t->changeArmMode((*reinterpret_cast< ArmMode(*)>(_a[1]))); break;
        case 5: _t->changeOpenLoopJoint((*reinterpret_cast< ArmJoint(*)>(_a[1]))); break;
        case 6: _t->changeCloseLoopMode((*reinterpret_cast< ArmClosedLoopMode(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (ArmController::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ArmController::motorEnableChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (ArmController::*_t)(ArmMode );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ArmController::armModeChanged)) {
                *result = 1;
            }
        }
        {
            typedef void (ArmController::*_t)(ArmJoint );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&ArmController::armJointChanged)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject ArmController::staticMetaObject = {
    { &JoystickInterface::staticMetaObject, qt_meta_stringdata_ArmController.data,
      qt_meta_data_ArmController,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ArmController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ArmController::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ArmController.stringdata0))
        return static_cast<void*>(const_cast< ArmController*>(this));
    return JoystickInterface::qt_metacast(_clname);
}

int ArmController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = JoystickInterface::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void ArmController::motorEnableChanged(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ArmController::armModeChanged(ArmMode _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ArmController::armJointChanged(ArmJoint _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE

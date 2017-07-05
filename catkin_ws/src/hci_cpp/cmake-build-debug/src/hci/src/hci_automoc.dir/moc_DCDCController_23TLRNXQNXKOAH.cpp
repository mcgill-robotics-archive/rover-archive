/****************************************************************************
** Meta object code from reading C++ file 'DCDCController.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/hci/src/controller/DCDCController.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DCDCController.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_DCDCController_t {
    QByteArrayData data[9];
    char stringdata0[144];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DCDCController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DCDCController_t qt_meta_stringdata_DCDCController = {
    {
QT_MOC_LITERAL(0, 0, 14), // "DCDCController"
QT_MOC_LITERAL(1, 15, 19), // "InputVoltageUpdated"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 19), // "InputCurrentUpdated"
QT_MOC_LITERAL(4, 56, 20), // "OutputVoltageUpdated"
QT_MOC_LITERAL(5, 77, 20), // "OutputCurrentUpdated"
QT_MOC_LITERAL(6, 98, 18), // "OutputPowerUpdated"
QT_MOC_LITERAL(7, 117, 18), // "TemperatureUpdated"
QT_MOC_LITERAL(8, 136, 7) // "process"

    },
    "DCDCController\0InputVoltageUpdated\0\0"
    "InputCurrentUpdated\0OutputVoltageUpdated\0"
    "OutputCurrentUpdated\0OutputPowerUpdated\0"
    "TemperatureUpdated\0process"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DCDCController[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       3,    1,   52,    2, 0x06 /* Public */,
       4,    1,   55,    2, 0x06 /* Public */,
       5,    1,   58,    2, 0x06 /* Public */,
       6,    1,   61,    2, 0x06 /* Public */,
       7,    1,   64,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    0,   67,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void DCDCController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DCDCController *_t = static_cast<DCDCController *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->InputVoltageUpdated((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 1: _t->InputCurrentUpdated((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->OutputVoltageUpdated((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->OutputCurrentUpdated((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->OutputPowerUpdated((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->TemperatureUpdated((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: _t->process(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DCDCController::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DCDCController::InputVoltageUpdated)) {
                *result = 0;
            }
        }
        {
            typedef void (DCDCController::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DCDCController::InputCurrentUpdated)) {
                *result = 1;
            }
        }
        {
            typedef void (DCDCController::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DCDCController::OutputVoltageUpdated)) {
                *result = 2;
            }
        }
        {
            typedef void (DCDCController::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DCDCController::OutputCurrentUpdated)) {
                *result = 3;
            }
        }
        {
            typedef void (DCDCController::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DCDCController::OutputPowerUpdated)) {
                *result = 4;
            }
        }
        {
            typedef void (DCDCController::*_t)(double );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DCDCController::TemperatureUpdated)) {
                *result = 5;
            }
        }
    }
}

const QMetaObject DCDCController::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_DCDCController.data,
      qt_meta_data_DCDCController,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *DCDCController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DCDCController::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_DCDCController.stringdata0))
        return static_cast<void*>(const_cast< DCDCController*>(this));
    return QObject::qt_metacast(_clname);
}

int DCDCController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
void DCDCController::InputVoltageUpdated(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DCDCController::InputCurrentUpdated(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void DCDCController::OutputVoltageUpdated(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void DCDCController::OutputCurrentUpdated(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void DCDCController::OutputPowerUpdated(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void DCDCController::TemperatureUpdated(double _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE

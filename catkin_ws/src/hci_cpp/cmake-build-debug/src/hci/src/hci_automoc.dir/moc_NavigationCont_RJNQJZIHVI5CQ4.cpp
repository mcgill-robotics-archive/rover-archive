/****************************************************************************
** Meta object code from reading C++ file 'NavigationController.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/hci/src/controller/NavigationController.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'NavigationController.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_NavigationController_t {
    QByteArrayData data[9];
    char stringdata0[105];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NavigationController_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NavigationController_t qt_meta_stringdata_NavigationController = {
    {
QT_MOC_LITERAL(0, 0, 20), // "NavigationController"
QT_MOC_LITERAL(1, 21, 12), // "pitchChanged"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 5), // "value"
QT_MOC_LITERAL(4, 41, 11), // "rollChanged"
QT_MOC_LITERAL(5, 53, 10), // "yawChanged"
QT_MOC_LITERAL(6, 64, 16), // "longitudeChanged"
QT_MOC_LITERAL(7, 81, 15), // "latitudeChanged"
QT_MOC_LITERAL(8, 97, 7) // "process"

    },
    "NavigationController\0pitchChanged\0\0"
    "value\0rollChanged\0yawChanged\0"
    "longitudeChanged\0latitudeChanged\0"
    "process"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NavigationController[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       4,    1,   47,    2, 0x06 /* Public */,
       5,    1,   50,    2, 0x06 /* Public */,
       6,    1,   53,    2, 0x06 /* Public */,
       7,    1,   56,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    0,   59,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void NavigationController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        NavigationController *_t = static_cast<NavigationController *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->pitchChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: _t->rollChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 2: _t->yawChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 3: _t->longitudeChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->latitudeChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->process(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (NavigationController::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&NavigationController::pitchChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (NavigationController::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&NavigationController::rollChanged)) {
                *result = 1;
            }
        }
        {
            typedef void (NavigationController::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&NavigationController::yawChanged)) {
                *result = 2;
            }
        }
        {
            typedef void (NavigationController::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&NavigationController::longitudeChanged)) {
                *result = 3;
            }
        }
        {
            typedef void (NavigationController::*_t)(float );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&NavigationController::latitudeChanged)) {
                *result = 4;
            }
        }
    }
}

const QMetaObject NavigationController::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_NavigationController.data,
      qt_meta_data_NavigationController,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *NavigationController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NavigationController::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_NavigationController.stringdata0))
        return static_cast<void*>(const_cast< NavigationController*>(this));
    return QObject::qt_metacast(_clname);
}

int NavigationController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void NavigationController::pitchChanged(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void NavigationController::rollChanged(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void NavigationController::yawChanged(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void NavigationController::longitudeChanged(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void NavigationController::latitudeChanged(float _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_END_MOC_NAMESPACE

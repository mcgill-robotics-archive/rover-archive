/****************************************************************************
** Meta object code from reading C++ file 'PoseDisplay.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/hci/src/view/PoseDisplay.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PoseDisplay.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PoseDisplay_t {
    QByteArrayData data[8];
    char stringdata0[68];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PoseDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PoseDisplay_t qt_meta_stringdata_PoseDisplay = {
    {
QT_MOC_LITERAL(0, 0, 11), // "PoseDisplay"
QT_MOC_LITERAL(1, 12, 8), // "setPitch"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 5), // "value"
QT_MOC_LITERAL(4, 28, 7), // "setRoll"
QT_MOC_LITERAL(5, 36, 6), // "setYaw"
QT_MOC_LITERAL(6, 43, 12), // "setLongitude"
QT_MOC_LITERAL(7, 56, 11) // "setLatitude"

    },
    "PoseDisplay\0setPitch\0\0value\0setRoll\0"
    "setYaw\0setLongitude\0setLatitude"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PoseDisplay[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x0a /* Public */,
       4,    1,   42,    2, 0x0a /* Public */,
       5,    1,   45,    2, 0x0a /* Public */,
       6,    1,   48,    2, 0x0a /* Public */,
       7,    1,   51,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,
    QMetaType::Void, QMetaType::Float,    3,

       0        // eod
};

void PoseDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PoseDisplay *_t = static_cast<PoseDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setPitch((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: _t->setRoll((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 2: _t->setYaw((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 3: _t->setLongitude((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->setLatitude((*reinterpret_cast< float(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject PoseDisplay::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PoseDisplay.data,
      qt_meta_data_PoseDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PoseDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PoseDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PoseDisplay.stringdata0))
        return static_cast<void*>(const_cast< PoseDisplay*>(this));
    return QWidget::qt_metacast(_clname);
}

int PoseDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

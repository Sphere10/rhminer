/**
 * rhminer base type functions
 *
 * Copyright 2018 Polyminer1 <https://github.com/polyminer1>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright
 * and related and neighboring rights to this software to the public domain
 * worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with
 * this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

///
/// @file
/// @copyright Polyminer1, QualiaLibre

#ifndef RHMINER_MICRORTTI_H
#define RHMINER_MICRORTTI_H

#include "corelib/basetypes.h"

#ifdef _DEBUG
#define RH_RTTI_ASSERT(A) RHMINER_ASSERT(A);
#else 
#define RH_RTTI_ASSERT(A)
#endif

#define RH_MICRO_RTTI(CLASS_NAME, PARENT, IS_ROOT)  public: struct _##CLASS_NAME##_MicropRTTI_Init \
    {  \
        enum E { ctTypeVal = RH_crc32_rec(0xFFFFFFFF, #CLASS_NAME) };  \
        unsigned    m_typeVal = E::ctTypeVal; \
        const char* m_typeName = #CLASS_NAME;  \
        bool isRoot = IS_ROOT;  \
        static _##CLASS_NAME##_MicropRTTI_Init ms_type;  \
    };  \
    typedef PARENT Super; \
    virtual unsigned    GetObjectType()       { return _##CLASS_NAME##_MicropRTTI_Init::ms_type.m_typeVal; }  \
    virtual unsigned    GetObjectType() const { return _##CLASS_NAME##_MicropRTTI_Init::ms_type.m_typeVal; }  \
    virtual const char* GetObjectTypeName()   { return #CLASS_NAME; }  \
    static  unsigned    GetType()             { return _##CLASS_NAME##_MicropRTTI_Init::ms_type.m_typeVal; }   \
    static  const char* GetTypeName()         { return #CLASS_NAME; } \
    static  bool        IsRoot()              { return _##CLASS_NAME##_MicropRTTI_Init::ms_type.isRoot; } \
    virtual BaseRTTI* DynamicCast(unsigned typeID) const \
    { \
        if (GetType() == typeID) \
            return (BaseRTTI*)this; \
        else \
            return PARENT::DynamicCast(typeID); \
        return (BaseRTTI*)NULL; \
    } \
    template <typename T> \
    T* as() \
    { \
        BaseRTTI* ptr = DynamicCast(T::GetType()); \
        RHMINER_ASSERT(ptr); \
        return static_cast<T*>(ptr); \
    } \
    template <typename T> \
    T* as() const \
    { \
        BaseRTTI* ptr = DynamicCast(T::GetType()); \
        RHMINER_ASSERT(ptr); \
        return static_cast<T*>(ptr); \
    } \

#define MICRO_RTTI_ROOT(CLASS_NAME)  RH_MICRO_RTTI(CLASS_NAME, BaseRTTI, true)
#define MICRO_RTTI(CLASS_NAME, PARENT)   RH_MICRO_RTTI(CLASS_NAME, PARENT, false)

#define MICRO_RTTI_DEFINE(CLASS_NAME) CLASS_NAME::_##CLASS_NAME##_MicropRTTI_Init CLASS_NAME::_##CLASS_NAME##_MicropRTTI_Init::ms_type;
#define MICRO_RTTI_DEFINE_TEMPL(TEMPLATE_NAME, CLASS_NAME) TEMPLATE_NAME<CLASS_NAME>::_##CLASS_NAME##_MicropRTTI_Init TEMPLATE_NAME<CLASS_NAME>::_##CLASS_NAME##_MicropRTTI_Init::ms_type;



// cpu base types
#if !defined(RHMINER_PLATFORM_GPU) && !defined(RANDOMHASH_CUDA)

    class BaseRTTI
    {
    public:
        virtual BaseRTTI* DynamicCast(unsigned typeID) const { return 0; }
    };

#endif  //!defined(RHMINER_PLATFORM_GPU) && !defined(RANDOMHASH_CUDA)

#endif //RHMINER_MICRORTTI_H




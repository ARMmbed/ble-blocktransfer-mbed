/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MBED_FUNCTIONPOINTER_WITH_CONTEXT_AND_RETURN_VALUE_H
#define MBED_FUNCTIONPOINTER_WITH_CONTEXT_AND_RETURN_VALUE_H

#include <string.h>


/** A class for storing and calling a pointer to a static or member function
 *  which takes a context.
 */
template <typename ContextType, typename ReturnType>
class FunctionPointerWithContextAndReturnValue {
public:
    typedef FunctionPointerWithContextAndReturnValue<ContextType, ReturnType> *pFunctionPointerWithContextAndReturnValue_t;
    typedef ReturnType (*pvoidfcontext_t)(ContextType context);

    /** Create a FunctionPointerWithContextAndReturnValue, attaching a static function
     *
     *  @param function The static function to attach (default is none)
     */
    FunctionPointerWithContextAndReturnValue(ReturnType (*function)(ContextType context) = NULL, ReturnType _default = NULL) :
        _function(NULL), _object(NULL), _member(), _membercaller(NULL), returnTypeNull(_default) {
        attach(function);
    }

    /** Create a FunctionPointerWithContextAndReturnValue, attaching a member function
     *
     *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
     *  @param function The address of the member function to attach
     */
    template<typename T>
    FunctionPointerWithContextAndReturnValue(T *object, ReturnType (T::*member)(ContextType context), ReturnType _default) :
        _function(NULL), _object(NULL), _member(), _membercaller(NULL), returnTypeNull(_default) {
        attach(object, member);
    }

    /** Attach a static function
     *
     *  @param function The static function to attach (default is none)
     */
    void attach(ReturnType (*function)(ContextType context) = NULL) {
        _function = function;
        _object = NULL;
    }

    /** Attach a member function
     *
     *  @param object The object pointer to invoke the member function on (i.e. the this pointer)
     *  @param function The address of the member function to attach
     */
    template<typename T>
    void attach(T *object, ReturnType (T::*member)(ContextType context)) {
        _object = static_cast<void *>(object);
        memcpy(_member, (char *)&member, sizeof(member));
        _membercaller = &FunctionPointerWithContextAndReturnValue::membercaller<T>;
        _function = NULL;
    }

    /** Call the attached static or member function; and if there are chained
     *  FunctionPointers their callbacks are invoked as well.
     *  @Note: all chained callbacks stack up; so hopefully there won't be too
     *  many FunctionPointers in a chain. */
    ReturnType call(ContextType context)
    {
        ReturnType retval = returnTypeNull;

        if (_function) {
            retval = _function(context);
        } else if (_object && _membercaller) {
            retval = _membercaller(_object, _member, context);
        }

        return retval;
    }

private:
    template<typename T>
    static ReturnType membercaller(void *object, char *member, ContextType context) {
        T *o = static_cast<T *>(object);
        ReturnType (T::*m)(ContextType);
        memcpy((char *)&m, member, sizeof(m));
        return (o->*m)(context);
    }

    ReturnType (*_function)(ContextType context);             /**< static function pointer - NULL if none attached */
    void *_object;                                      /**< object this pointer - NULL if none attached */
    char _member[16];                                   /**< raw member function pointer storage - converted back by
                                                         *   registered _membercaller */
    ReturnType (*_membercaller)(void *, char *, ContextType); /**< registered membercaller function to convert back and call
                                                         *   _member on _object passing the context. */
    ReturnType returnTypeNull;
};

#endif // ifndef MBED_FUNCTIONPOINTER_WITH_CONTEXT_AND_RETURN_VALUE_H

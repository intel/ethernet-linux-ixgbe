/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 1999 - 2025 Intel Corporation */

/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _KCOMPAT_CLEANUP_H_
#define _KCOMPAT_CLEANUP_H_

#include <linux/compiler.h>

#ifndef typeof_member
#define typeof_member(T, m)     typeof(((T*)0)->m)
#endif
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000		\
		     + __GNUC_MINOR__ * 100	\
		     + __GNUC_PATCHLEVEL__)
#endif /* GCC_VERSION */
#define GCC_IS_BELOW(x) (GCC_VERSION < (x))

#if defined __has_attribute
#if __has_attribute(cleanup)
#define HAVE_ATTRIBUTE_CLEANUP
#endif
#else
#if !GCC_IS_BELOW(30306)
#define HAVE_ATTRIBUTE_CLEANUP
#endif
#endif

#ifndef HAVE_ATTRIBUTE_CLEANUP
#error "This compiler is too old, cleanup extension (GCC 3.3.6) is required."
#endif

#if defined(__clang__)
/*
 * Clang prior to 17 is being silly and considers many __cleanup() variables
 * as unused (because they are, their sole purpose is to go out of scope).
 *
 * https://reviews.llvm.org/D152180
 */
#define __cleanup(func) __maybe_unused __attribute__((__cleanup__(func)))
#else /* __clang__ */
/*
 * gcc: https://gcc.gnu.org/onlinedocs/gcc/Common-Variable-Attributes.html#index-cleanup-variable-attribute
 * clang: https://clang.llvm.org/docs/AttributeReference.html#cleanup
 */
#define __cleanup(func)                 __attribute__((__cleanup__(func)))
#endif /* __clang__ */

/*
 * DEFINE_FREE(name, type, free):
 *	simple helper macro that defines the required wrapper for a __free()
 *	based cleanup function. @free is an expression using '_T' to access the
 *	variable. @free should typically include a NULL test before calling a
 *	function, see the example below.
 *
 * __free(name):
 *	variable attribute to add a scoped based cleanup to the variable.
 *
 * no_free_ptr(var):
 *	like a non-atomic xchg(var, NULL), such that the cleanup function will
 *	be inhibited -- provided it sanely deals with a NULL value.
 *
 *	NOTE: this has __must_check semantics so that it is harder to accidentally
 *	leak the resource.
 *
 * return_ptr(p):
 *	returns p while inhibiting the __free().
 *
 * Ex.
 *
 * DEFINE_FREE(kfree, void *, if (_T) kfree(_T))
 *
 * void *alloc_obj(...)
 * {
 *	struct obj *p __free(kfree) = kmalloc(...);
 *	if (!p)
 *		return NULL;
 *
 *	if (!init_obj(p))
 *		return NULL;
 *
 *	return_ptr(p);
 * }
 *
 * NOTE: the DEFINE_FREE()'s @free expression includes a NULL test even though
 * kfree() is fine to be called with a NULL value. This is on purpose. This way
 * the compiler sees the end of our alloc_obj() function as:
 *
 *	tmp = p;
 *	p = NULL;
 *	if (p)
 *		kfree(p);
 *	return tmp;
 *
 * And through the magic of value-propagation and dead-code-elimination, it
 * eliminates the actual cleanup call and compiles into:
 *
 *	return p;
 *
 * Without the NULL test it turns into a mess and the compiler can't help us.
 */

#define DEFINE_FREE(_name, _type, _free) \
	static inline void __free_##_name(void *p) { _type _T = *(_type *)p; _free; }

#define __free(_name)	__cleanup(__free_##_name)

#define __get_and_null_ptr(p) \
	({ __auto_type __ptr = &(p); \
	   __auto_type __val = *__ptr; \
	   *__ptr = NULL;  __val; })

static inline __must_check
const volatile void * __must_check_fn(const volatile void *val)
{ return val; }

#define no_free_ptr(p) \
	((typeof(p)) __must_check_fn(__get_and_null_ptr(p)))

#define return_ptr(p)	return no_free_ptr(p)


/*
 * DEFINE_CLASS(name, type, exit, init, init_args...):
 *	helper to define the destructor and constructor for a type.
 *	@exit is an expression using '_T' -- similar to FREE above.
 *	@init is an expression in @init_args resulting in @type
 *
 * EXTEND_CLASS(name, ext, init, init_args...):
 *	extends class @name to @name@ext with the new constructor
 *
 * CLASS(name, var)(args...):
 *	declare the variable @var as an instance of the named class
 *
 * Ex.
 *
 * DEFINE_CLASS(fdget, struct fd, fdput(_T), fdget(fd), int fd)
 *
 *	CLASS(fdget, f)(fd);
 *	if (!f.file)
 *		return -EBADF;
 *
 *	// use 'f' without concern
 */

#define DEFINE_CLASS(_name, _type, _exit, _init, _init_args...)		\
typedef _type class_##_name##_t;					\
static inline void class_##_name##_destructor(_type *p)			\
{ _type _T = *p; _exit; }						\
static inline _type class_##_name##_constructor(_init_args)		\
{ _type t = _init; return t; }

#define EXTEND_CLASS(_name, ext, _init, _init_args...)			\
typedef class_##_name##_t class_##_name##ext##_t;			\
static inline void class_##_name##ext##_destructor(class_##_name##_t *p)\
{ class_##_name##_destructor(p); }					\
static inline class_##_name##_t class_##_name##ext##_constructor(_init_args) \
{ class_##_name##_t t = _init; return t; }

#define CLASS(_name, var)						\
	class_##_name##_t var __cleanup(class_##_name##_destructor) =	\
		class_##_name##_constructor

/*
 * DEFINE_GUARD(name, type, lock, unlock):
 *	trivial wrapper around DEFINE_CLASS() above specifically
 *	for locks.
 *
 * DEFINE_GUARD_COND(name, ext, condlock)
 *	wrapper around EXTEND_CLASS above to add conditional lock
 *	variants to a base class, eg. mutex_trylock() or
 *	mutex_lock_interruptible().
 *
 * guard(name):
 *	an anonymous instance of the (guard) class, not recommended for
 *	conditional locks.
 *
 * scoped_guard (name, args...) { }:
 *	similar to CLASS(name, scope)(args), except the variable (with the
 *	explicit name 'scope') is declard in a for-loop such that its scope is
 *	bound to the next (compound) statement.
 *
 *	for conditional locks the loop body is skipped when the lock is not
 *	acquired.
 *
 * scoped_cond_guard (name, fail, args...) { }:
 *      similar to scoped_guard(), except it does fail when the lock
 *      acquire fails.
 *
 */

#define DEFINE_GUARD(_name, _type, _lock, _unlock) \
	DEFINE_CLASS(_name, _type, if (_T) { _unlock; }, ({ _lock; _T; }), _type _T); \
	static inline void * class_##_name##_lock_ptr(class_##_name##_t *_T) \
	{ return *_T; }

#define DEFINE_GUARD_COND(_name, _ext, _condlock) \
	EXTEND_CLASS(_name, _ext, \
		     ({ void *_t = _T; if (_T && !(_condlock)) _t = NULL; _t; }), \
		     class_##_name##_t _T) \
	static inline void * class_##_name##_ext##_lock_ptr(class_##_name##_t *_T) \
	{ return class_##_name##_lock_ptr(_T); }

#define guard(_name) \
	CLASS(_name, __UNIQUE_ID(guard))

#define __guard_ptr(_name) class_##_name##_lock_ptr

#define scoped_guard(_name, args...)					\
	for (CLASS(_name, scope)(args),					\
	     *done = NULL; __guard_ptr(_name)(&scope) && !done; done = (void *)1)

#define scoped_cond_guard(_name, _fail, args...) \
	for (CLASS(_name, scope)(args), \
	     *done = NULL; !done; done = (void *)1) \
		if (!__guard_ptr(_name)(&scope)) _fail; \
		else

/*
 * Additional helper macros for generating lock guards with types, either for
 * locks that don't have a native type (eg. RCU, preempt) or those that need a
 * 'fat' pointer (eg. spin_lock_irqsave).
 *
 * DEFINE_LOCK_GUARD_0(name, lock, unlock, ...)
 * DEFINE_LOCK_GUARD_1(name, type, lock, unlock, ...)
 * DEFINE_LOCK_GUARD_1_COND(name, ext, condlock)
 *
 * will result in the following type:
 *
 *   typedef struct {
 *	type *lock;		// 'type := void' for the _0 variant
 *	__VA_ARGS__;
 *   } class_##name##_t;
 *
 * As above, both _lock and _unlock are statements, except this time '_T' will
 * be a pointer to the above struct.
 */

#define __DEFINE_UNLOCK_GUARD(_name, _type, _unlock, ...)		\
typedef struct {							\
	_type *lock;							\
	__VA_ARGS__;							\
} class_##_name##_t;							\
									\
static inline void class_##_name##_destructor(class_##_name##_t *_T)	\
{									\
	if (_T->lock) { _unlock; }					\
}									\
									\
static inline void *class_##_name##_lock_ptr(class_##_name##_t *_T)	\
{									\
	return _T->lock;						\
}


#define __DEFINE_LOCK_GUARD_1(_name, _type, _lock)			\
static inline class_##_name##_t class_##_name##_constructor(_type *l)	\
{									\
	class_##_name##_t _t = { .lock = l }, *_T = &_t;		\
	_lock;								\
	return _t;							\
}

#define __DEFINE_LOCK_GUARD_0(_name, _lock)				\
static inline class_##_name##_t class_##_name##_constructor(void)	\
{									\
	class_##_name##_t _t = { .lock = (void*)1 },			\
			 *_T __maybe_unused = &_t;			\
	_lock;								\
	return _t;							\
}

#define DEFINE_LOCK_GUARD_1(_name, _type, _lock, _unlock, ...)		\
__DEFINE_UNLOCK_GUARD(_name, _type, _unlock, __VA_ARGS__)		\
__DEFINE_LOCK_GUARD_1(_name, _type, _lock)

#define DEFINE_LOCK_GUARD_0(_name, _lock, _unlock, ...)			\
__DEFINE_UNLOCK_GUARD(_name, void, _unlock, __VA_ARGS__)		\
__DEFINE_LOCK_GUARD_0(_name, _lock)

#define DEFINE_LOCK_GUARD_1_COND(_name, _ext, _condlock)		\
	EXTEND_CLASS(_name, _ext,					\
		     ({ class_##_name##_t _t = { .lock = l }, *_T = &_t;\
		        if (_T->lock && !(_condlock)) _T->lock = NULL;	\
			_t; }),						\
		     typeof_member(class_##_name##_t, lock) l)		\
	static inline void * class_##_name##_ext##_lock_ptr(class_##_name##_t *_T) \
	{ return class_##_name##_lock_ptr(_T); }

#include <linux/mutex.h>

DEFINE_GUARD(mutex, struct mutex *, mutex_lock(_T), mutex_unlock(_T))
DEFINE_FREE(mutex, struct mutex *, if (_T) mutex_unlock(_T))

#include <linux/slab.h>

DEFINE_FREE(kfree, void *, if (_T) kfree(_T))

#include <linux/spinlock.h>

DEFINE_LOCK_GUARD_1(raw_spinlock, raw_spinlock_t,
		    raw_spin_lock(_T->lock),
		    raw_spin_unlock(_T->lock))

DEFINE_LOCK_GUARD_1_COND(raw_spinlock, _try, raw_spin_trylock(_T->lock))

DEFINE_LOCK_GUARD_1(raw_spinlock_nested, raw_spinlock_t,
		    raw_spin_lock_nested(_T->lock, SINGLE_DEPTH_NESTING),
		    raw_spin_unlock(_T->lock))

DEFINE_LOCK_GUARD_1(raw_spinlock_irq, raw_spinlock_t,
		    raw_spin_lock_irq(_T->lock),
		    raw_spin_unlock_irq(_T->lock))

DEFINE_LOCK_GUARD_1_COND(raw_spinlock_irq, _try, raw_spin_trylock_irq(_T->lock))

DEFINE_LOCK_GUARD_1(raw_spinlock_irqsave, raw_spinlock_t,
		    raw_spin_lock_irqsave(_T->lock, _T->flags),
		    raw_spin_unlock_irqrestore(_T->lock, _T->flags),
		    unsigned long flags)

DEFINE_LOCK_GUARD_1_COND(raw_spinlock_irqsave, _try,
			 raw_spin_trylock_irqsave(_T->lock, _T->flags))

DEFINE_LOCK_GUARD_1(spinlock, spinlock_t,
		    spin_lock(_T->lock),
		    spin_unlock(_T->lock))

DEFINE_LOCK_GUARD_1_COND(spinlock, _try, spin_trylock(_T->lock))

DEFINE_LOCK_GUARD_1(spinlock_irq, spinlock_t,
		    spin_lock_irq(_T->lock),
		    spin_unlock_irq(_T->lock))

DEFINE_LOCK_GUARD_1_COND(spinlock_irq, _try,
			 spin_trylock_irq(_T->lock))

DEFINE_LOCK_GUARD_1(spinlock_irqsave, spinlock_t,
		    spin_lock_irqsave(_T->lock, _T->flags),
		    spin_unlock_irqrestore(_T->lock, _T->flags),
		    unsigned long flags)

DEFINE_LOCK_GUARD_1_COND(spinlock_irqsave, _try,
			 spin_trylock_irqsave(_T->lock, _T->flags))

DEFINE_LOCK_GUARD_1(read_lock, rwlock_t,
		    read_lock(_T->lock),
		    read_unlock(_T->lock))

DEFINE_LOCK_GUARD_1(read_lock_irq, rwlock_t,
		    read_lock_irq(_T->lock),
		    read_unlock_irq(_T->lock))

DEFINE_LOCK_GUARD_1(read_lock_irqsave, rwlock_t,
		    read_lock_irqsave(_T->lock, _T->flags),
		    read_unlock_irqrestore(_T->lock, _T->flags),
		    unsigned long flags)

DEFINE_LOCK_GUARD_1(write_lock, rwlock_t,
		    write_lock(_T->lock),
		    write_unlock(_T->lock))

DEFINE_LOCK_GUARD_1(write_lock_irq, rwlock_t,
		    write_lock_irq(_T->lock),
		    write_unlock_irq(_T->lock))

DEFINE_LOCK_GUARD_1(write_lock_irqsave, rwlock_t,
		    write_lock_irqsave(_T->lock, _T->flags),
		    write_unlock_irqrestore(_T->lock, _T->flags),
		    unsigned long flags)

#include <linux/rcupdate.h>

DEFINE_LOCK_GUARD_0(rcu, rcu_read_lock(), rcu_read_unlock())

#endif /* _KCOMPAT_CLENAUP_H_ */

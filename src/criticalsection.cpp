#include "criticalsection.h"

#if defined _WIN32
# include <windows.h>
#elif defined __unix__
# include <pthread.h>
#endif

namespace dymp{
namespace mpc{

///////////////////////////////////////////////////////////////////////////////////////////////////

class CriticalSectionImpl{
public:
#if defined _WIN32
	CRITICAL_SECTION	cs;
#elif defined __unix__
	pthread_mutex_t  mutex;
#endif

	CriticalSectionImpl(int spin){
#if defined _WIN32
		InitializeCriticalSectionAndSpinCount(&cs, spin);
#elif defined __unix__
		pthread_mutex_init(&mutex, 0);
#endif
	}

	~CriticalSectionImpl(){
#if defined _WIN32
		DeleteCriticalSection(&cs);
#elif defined __unix__
		pthread_mutex_destroy(&mutex);
#endif
	}

	void Enter(){
#if defined _WIN32
		EnterCriticalSection(&cs);
#elif defined __unix__
		pthread_mutex_lock(&mutex);
#endif
	}

	bool TryEnter(){
#ifdef _WIN32
		return (TryEnterCriticalSection(&cs) == TRUE);
#else
		return false;
#endif
	}

	void Leave(){
#if defined _WIN32
		LeaveCriticalSection(&cs);
#elif defined __unix__
		pthread_mutex_unlock(&mutex);
#endif
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

CriticalSection::CriticalSection(){
	target    = 0;
	spinCount = 0;
	impl      = 0;
	Create();
}

CriticalSection::CriticalSection(CriticalSection* _target){
	target = _target;
	target->Enter();
}

CriticalSection::~CriticalSection(){
	if(target)
		target->Leave();
	else
		Delete();
}

void CriticalSection::Create(){
	if(!impl)
		impl = new CriticalSectionImpl(spinCount);
}

void CriticalSection::Delete(){
	if(impl){
		delete impl;
		impl = 0;
	}
}

void CriticalSection::Enter(){
	impl->Enter();
}

bool CriticalSection::TryEnter(){
	return impl->TryEnter();
}

void CriticalSection::Leave(){
	impl->Leave();
}

}
}

#include "event.h"

#if defined _WIN32
# include <windows.h>
# define USE_WFMO 1
#elif defined __unix__
# include <pthread.h>
# define USE_WFMO 0
#endif

#include <iostream>
#include <algorithm>
using namespace std;

namespace dymp{
namespace mpc{

struct EventHandle{
#if defined _WIN32
	HANDLE  handle;
#elif defined __unix__
	pthread_mutex_t  mutex;
	pthread_cond_t   cond;
	bool value;
#endif
	bool manual;

	EventHandle(){
#if defined _WIN32
		handle = 0;
#elif defined __unix__
		value  = false;
#endif
		manual = false;
	}
};

typedef std::map<void*, EventHandle> EventHandles;
static EventHandles	eventHandles;

EventHandle* GetEventHandle(Event* ev){
	EventHandles::iterator it = eventHandles.find(ev);
	if(it != eventHandles.end())
		return &it->second;
	return 0;
}

Event::Event(){

}

Event::~Event(){
//	Close();
}

bool Event::Create(bool manual){
	Close();

	// create named event if one is given
	EventHandle h;
#if defined _WIN32
	h.handle = CreateEventA(0, manual, 0, (name.empty() ? 0 : name.c_str()));
	if(!h.handle)
		return false;
#elif defined __unix__
	if(pthread_cond_init(&h.cond, 0)){
		cout << "cond init failed" << endl;
		return false;
	}
	if(pthread_mutex_init(&h.mutex, 0)){
		cout << "mutex init failed" << endl;
		return false;
	}
#endif
	h.manual = manual;
	eventHandles[this] = h;
	return true;
}

void Event::Close(){
	EventHandle* h = GetEventHandle(this);
	if(!h)
		return;
#if defined _WIN32
	if(h->handle)
		CloseHandle(h->handle);
#elif defined __unix__
	pthread_cond_destroy(&h->cond);
	pthread_mutex_destroy(&h->mutex);
#endif
}

bool Event::Wait(int timeout){
	EventHandle* h = GetEventHandle(this);
	if(!h){
		Create();
		h = GetEventHandle(this);
	}
#if defined _WIN32
	int res = WaitForSingleObject(h->handle, timeout);
	// 取得成功
	if(res == WAIT_OBJECT_0)
		return true;
	// タイムアウト
	if(res == WAIT_TIMEOUT)
		return false;
	// エラー
	if(res == WAIT_FAILED)
		return false;
#elif defined __unix__
	const int _1e6 = 1000000;
	const int _1e9 = 1000000000;
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_nsec += _1e6 * timeout;
	if(ts.tv_nsec >= _1e9){
		ts.tv_sec++;
		ts.tv_nsec -= _1e9;
	}
	
	while(!h->value){
		//cout << "mutex locking..." << endl;
		if(pthread_mutex_lock(&h->mutex)){
			cout << "mutex lock failed" << endl;
			return false;
		}
		//cout << "mutex locked" << endl;
		//cout << "cond waiting..." << endl;
		int ret = pthread_cond_timedwait(&h->cond, &h->mutex, &ts);
		//cout << "mutex unlocking..." << endl;
		pthread_mutex_unlock(&h->mutex);
		if(ret == ETIMEDOUT){
			//cout << "cond wait timed out" << endl;
			return false;
		}
		if(ret == EINTR){
			//cout << "cond wait interrupted" << endl;
			return false;
		}
	}
	// reset here if this event is auto-reset
	if(!h->manual)
		h->value = false;
	
	//cout << "cond wait succeeded" << endl;	
	return true;
#endif
	return false;
}

void Event::Set(){
	//cout << "setting event" << endl;
	EventHandle* h = GetEventHandle(this);
	if(!h){
		Create();
		h = GetEventHandle(this);
	}
#if defined _WIN32
	SetEvent(h->handle);
#elif defined __unix__
	h->value = true;
	pthread_mutex_lock(&h->mutex);
	pthread_cond_broadcast(&h->cond);
	pthread_mutex_unlock(&h->mutex);
#endif

#if USE_WFMO != 1
	//cout << "setting groups " << groups.size() << endl;
	for(EventGroup* gr : groups){
		CriticalSection _cs(&gr->cs);
		gr->Set();
	}
#endif
}

bool Event::IsSet(){
#if defined _WIN32
	return Wait(0);
#elif defined __unix__
	EventHandle* h = GetEventHandle(this);
	if(h){
		bool ret = h->value;
		// reset here if this event is auto-reset
		if(!h->manual)
			h->value = false;
		return ret;
	}

	return false;
#endif
}

void Event::Reset(){
	EventHandle* h = GetEventHandle(this);
	if(!h){
		Create();
		h = GetEventHandle(this);
	}
#if defined _WIN32
	ResetEvent(h->handle);
#elif defined __unix__
	h->value = false;
#endif
}

EventGroup::EventGroup(){

}

EventGroup::~EventGroup(){
//	while(!empty())
//		Remove(at(0));
}

void EventGroup::Remove(Event* ev){
	CriticalSection _cs(&cs);
	CriticalSection _csEv(&ev->cs);

	vector<EventGroup*>::iterator it = std::find(ev->groups.begin(), ev->groups.end(), this);
	if(it != ev->groups.end())
		ev->groups.erase(it);

	erase(find(begin(), end(), ev));
}

void EventGroup::Add(Event* ev){
	CriticalSection _cs(&cs);
	CriticalSection _csEv(&ev->cs);

	push_back(ev);
	ev->groups.push_back(this);
}

int EventGroup::Wait(int timeout){
#if USE_WFMO == 1
	// implement using WaitForMultipleObjects
	vector<HANDLE>	handles;
	for(int i = 0; i < (int)size(); i++){
		EventHandle* h = GetEventHandle(at(i));
		if(!h){
			at(i)->Create();
			h = GetEventHandle(at(i));
		}
		handles.push_back(h->handle);
	}

	int res = WaitForMultipleObjects(size(), &handles[0], false, timeout);

	// event detected
	if(WAIT_OBJECT_0 <= res && res < WAIT_OBJECT_0 + size()){
		return res - WAIT_OBJECT_0;
	}
	// timed out
	if(res == WAIT_TIMEOUT)
		return -1;
	// error
	if(res == WAIT_FAILED)
		return -1;
#else
	if(!Event::Wait(timeout))
		return -1;

	for(int i = 0; i < (int)size(); i++){
		if(at(i)->IsSet())
			return i;
	}
#endif

	return -1;
}

}
}

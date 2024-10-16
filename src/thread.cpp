#include "thread.h"

#include <thread>

namespace dymp{
namespace mpc{

///////////////////////////////////////////////////////////////////////////////////////////////////

class ThreadImpl{
	std::thread	th;
public:
	void Run(Thread* func){
		th = std::thread(std::ref(*func));
	}
	bool Join(){
		th.join();
		return true;
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

Thread::Thread(){
	impl = new ThreadImpl();
}

Thread::~Thread(){
	delete impl;
}

void Thread::operator()(){
	Func();
}

void Thread::Run(){
	impl->Run(this);
}

bool Thread::Join(){
	return impl->Join();
}

}
}

#pragma once

namespace dymp{
namespace mpc{

/**
	wrapper of Windows Critical Section
 */

class CriticalSectionImpl;

class CriticalSection{
private:
	CriticalSection(const CriticalSection& ):impl(0), target(0), spinCount(0){}	 ///< disable calling copy constructor

protected:
	CriticalSectionImpl* impl;
	CriticalSection* target;

public:
	int	spinCount;			///< loop count before calling WaitForSingleObject

public:
	void Create  ();
	void Delete  ();
	void Enter   ();
	bool TryEnter();
	void Leave   ();
	
	 CriticalSection();

	 // entor in constructor, leave in destructor
	 CriticalSection(CriticalSection* _target);
	~CriticalSection();
};

}
}


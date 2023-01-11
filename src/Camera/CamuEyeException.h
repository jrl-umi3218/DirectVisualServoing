// Romain Marie
// MIS Laboratory, Amiens, FRANCE
// Mai 2012

#pragma once
#include <ueye.h>
#include <sstream>
#include <exception>
#include <iostream>

class CamuEyeException : public std::exception {
private:
        HIDS cam;
        int exceptionId;
public:
        CamuEyeException(HIDS cam, int err) {
                exceptionId = err;
        }
        const char * what() const throw () {
                std::stringstream ss;
         		 std::cout << "CamuEyeException on camera " << cam <<", with exit code:\t" << exceptionId << std::endl;

                return ss.str().c_str();
        }
        HIDS getCam();
        int getExceptionId();
};

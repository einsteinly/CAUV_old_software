#include "module.h"

#include <common/cauv_utils.h>


FTDIException::FTDIException(const std::string& msg) : message(msg) {}
FTDIException::FTDIException(const std::string& msg, int errCode, ftdi_context* ftdic)
{
    message = MakeString() << msg << ": " << errCode << " (" << ftdi_get_error_string(ftdic) << ")";
}
FTDIException::~FTDIException() throw() {}
const char* FTDIException::what() const throw()
{
	return message.c_str();
}

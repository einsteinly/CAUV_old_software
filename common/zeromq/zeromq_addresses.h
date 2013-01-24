/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef ZEROMQ_ADDRESSES_H
#define ZEROMQ_ADDRESSES_H
#include <string>
#include <vector>
#include <utility>
#include <stdint.h>

namespace cauv {

typedef std::pair<bool, std::vector<uint32_t> > subscription_vec_t;

std::string gen_subscription_message(subscription_vec_t);

subscription_vec_t parse_subscription_message(std::string msg);

std::string get_vehicle_name(void);

std::string get_ipc_directory(void);

std::string get_ipc_directory(const std::string vehicle_name);

}

#endif

//
// Copyright 2010 CNRS
//
// Author: Florent Lamiraux
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// dynamic-graph is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with dynamic-graph.  If not, see <http://www.gnu.org/licenses/>.

#ifndef DYNAMIC_GRAPH_TUTORIAL_COMMAND_INCREMENT_HH
#define DYNAMIC_GRAPH_TUTORIAL_COMMAND_INCREMENT_HH

#include <string>

#include <boost/assign/list_of.hpp>

#include <sot-stabilizer/prototyping/linearized-table-cart-device.hh>

namespace command
{
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;
class Increment : public Command
{
public:
    /// Create a command and store it in Entity
    /// \param entity Instance of Entity owning of the command
    /// \param docstring documentation of the command
    Increment (sotStabilizer::LinearizedTableCartDevice & entity,
        const std::string& docstring) :
            Command(entity,boost::assign::list_of(Value::DOUBLE), docstring)
    {
    }

    virtual ~Increment() {}



    virtual Value doExecute()
    {
        dynamicgraph::Entity& entity = owner();
        sotStabilizer::LinearizedTableCartDevice& ip =
                static_cast<sotStabilizer::LinearizedTableCartDevice&>(entity);
        std::vector<Value> values = getParameterValues();
        double timeStep = values[0].value();
        ip.incr(timeStep);
        return Value();
    }



}; //class Increment
} // namespace command

#endif //DYNAMIC_GRAPH_TUTORIAL_COMMAND_INCREMENT_HH


/* 
 * Copyright (c) [year], [author]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: [Your Name] <[Your Email]>
 */

#include "levy-flight-2d-mobility-model.h"

#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LevyFlight2d");
NS_OBJECT_ENSURE_REGISTERED(LevyFlight2dMobilityModel);

TypeId LevyFlight2dMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LevyFlight2dMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<LevyFlight2dMobilityModel>()
            .AddAttribute("Bounds",
                          "Bounds of the area to cruise.",
                          RectangleValue(Rectangle(0.0, 100.0, 0.0, 100.0)),
                          MakeRectangleAccessor(&LevyFlight2dMobilityModel::m_bounds),
                          MakeRectangleChecker())
            .AddAttribute("StepSize",
                          "Step size of the Levy flight.",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&LevyFlight2dMobilityModel::m_stepSize),
                          MakeDoubleChecker<double>())
            .AddAttribute("Alpha",
                          "Exponent parameter for the Levy flight distribution.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&LevyFlight2dMobilityModel::m_alpha),
                          MakeDoubleChecker<double>())
            .AddAttribute("Direction",
                          "A random variable used to pick the direction (radians).",
                          StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                          MakePointerAccessor(&LevyFlight2dMobilityModel::m_direction),
                          MakePointerChecker<RandomVariableStream>())
            .AddAttribute("Speed",
                          "A random variable used to pick the speed (m/s).",
                          StringValue("ns3::UniformRandomVariable[Min=2.0|Max=4.0]"),
                          MakePointerAccessor(&LevyFlight2dMobilityModel::m_speed),
                          MakePointerChecker<RandomVariableStream>());
    return tid;
}

void LevyFlight2dMobilityModel::DoInitialize()
{
    DoInitializePrivate();
    MobilityModel::DoInitialize();
}

void LevyFlight2dMobilityModel::DoInitializePrivate()
{
    m_helper.Update();
    Vector position = m_helper.GetCurrentPosition();

    double speed = m_speed->GetValue();
    double direction = m_direction->GetValue();

    Vector vector(std::cos(direction) * speed, std::sin(direction) * speed, 0.0);
    m_helper.SetVelocity(vector);
    m_helper.Unpause();

    // Levy flight implementation
    double stepLength = m_stepSize * std::pow(m_direction->GetValue(), -1.0 / m_alpha);
    Vector newPosition = position;
    newPosition.x += vector.x * stepLength;
    newPosition.y += vector.y * stepLength;

    std::cout << newPosition.x << " " << newPosition.y << "\n";

    if (m_bounds.IsInside(newPosition))
    {
        m_helper.SetPosition(newPosition);
        NotifyCourseChange();
    }
    else
    {
        // If the new position is outside bounds, discard the step and generate a new one
        DoInitializePrivate();
    }
}

void LevyFlight2dMobilityModel::DoDispose()
{
    // chain up
    MobilityModel::DoDispose();
}

Vector LevyFlight2dMobilityModel::DoGetPosition() const
{
    m_helper.UpdateWithBounds(m_bounds);
    return m_helper.GetCurrentPosition();
}

void LevyFlight2dMobilityModel::DoSetPosition(const Vector &position)
{
    NS_ASSERT(m_bounds.IsInside(position));
    m_helper.SetPosition(position);
    m_event.Cancel();
    m_event = Simulator::ScheduleNow(&LevyFlight2dMobilityModel::DoInitializePrivate, this);
}

Vector LevyFlight2dMobilityModel::DoGetVelocity() const
{
    return m_helper.GetVelocity();
}

int64_t LevyFlight2dMobilityModel::DoAssignStreams(int64_t stream)
{
    m_speed->SetStream(stream);
    m_direction->SetStream(stream + 1);
    return 2;
}

} // namespace ns3


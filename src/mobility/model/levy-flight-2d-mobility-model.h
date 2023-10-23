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
#ifndef LEVY_FLIGHT_2D_MOBILITY_MODEL_H
#define LEVY_FLIGHT_2D_MOBILITY_MODEL_H

#include "mobility-model.h"
#include "rectangle.h"

#include "ns3/random-variable-stream.h"
#include "ns3/constant-velocity-helper.h"

namespace ns3
{

class LevyFlight2dMobilityModel : public MobilityModel
{
public:
    static TypeId GetTypeId();

private:
    void Rebound(Time timeLeft);
    void DoWalk(Time delayLeft);
    void DoInitializePrivate();
    void DoDispose() override;
    void DoInitialize() override;
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector &position) override;
    Vector DoGetVelocity() const override;
    int64_t DoAssignStreams(int64_t) override;

    ConstantVelocityHelper m_helper;
    EventId m_event;
    Ptr<RandomVariableStream> m_stepSize;
    double m_alpha;
    Time m_modeTime;
    Ptr<RandomVariableStream> m_speed;
    Ptr<RandomVariableStream> m_direction;
    Rectangle m_bounds;
};

} // namespace ns3

#endif /* LEVY_FLIGHT_2D_MOBILITY_MODEL_H */

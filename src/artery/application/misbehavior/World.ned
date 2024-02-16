package artery.application.misbehavior;

import artery.envmod.World;
import artery.application.misbehavior.authority.MisbehaviorAuthorityCpm;

network World extends artery.envmod.World
{

    submodules:
        misbehaviorAuthority: MisbehaviorAuthorityCpm {
        }

}
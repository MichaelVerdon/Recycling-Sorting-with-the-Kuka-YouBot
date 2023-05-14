function createPath(name,startPoint,startOrient,endPoint,endOrient)
    -- Create Path Object
    local path = sim.createPath(1)

    -- Create buffer variables
    local buffer = {startPoint[1],startPoint[2],startPoint[3],startOrient[1],startOrient[2],startOrient[3], 1,0,0,0,0,
                    endPoint[1],endPoint[2],endPoint[3],endOrient[1],endOrient[2],endOrient[3],             1,0,0,0,0}

    -- Insert 2 control points (start and endpoint)
    sim.insertPathCtrlPoints(path,0,0,2,buffer)

    -- Rename the object
    sim.setObjectName(path,name)

    -- Return handle to path
    return path
end

function sysCall_init()
         
    pathHandle=sim.getObjectHandle("ConveyorBeltPath#0")
    forwarder=sim.getObjectHandle('ConveyorBelt_forwarder#0')
    sensor=sim.getObjectHandle('ConveyorBelt_sensor#0')
    sim.setPathTargetNominalVelocity(pathHandle,0) -- for backward compatibility

end

function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    
    beltVelocity=sim.getScriptSimulationParameter(sim.handle_self,"conveyorBeltVelocity")
    if (sim.readProximitySensor(sensor)>0) then
        beltVelocity=0
    end
    local dt=sim.getSimulationTimeStep()
    local pos=sim.getPathPosition(pathHandle)
    pos=pos+beltVelocity*dt
    sim.setPathPosition(pathHandle,pos) -- update the path's intrinsic position
    
    
    -- Here we "fake" the transportation pads with a single static rectangle that we dynamically reset
    -- at each simulation pass (while not forgetting to set its initial velocity vector) :
    
    relativeLinearVelocity={beltVelocity,0,0}
    -- Reset the dynamic rectangle from the simulation (it will be removed and added again)
    sim.resetDynamicObject(forwarder)
    -- Compute the absolute velocity vector:
    m=sim.getObjectMatrix(forwarder,-1)
    m[4]=0 -- Make sure the translation component is discarded
    m[8]=0 -- Make sure the translation component is discarded
    m[12]=0 -- Make sure the translation component is discarded
    absoluteLinearVelocity=sim.multiplyVector(m,relativeLinearVelocity)
    -- Now set the initial velocity of the dynamic rectangle:
    sim.setObjectFloatParameter(forwarder,sim.shapefloatparam_init_velocity_x,absoluteLinearVelocity[1])
    sim.setObjectFloatParameter(forwarder,sim.shapefloatparam_init_velocity_y,absoluteLinearVelocity[2])
    sim.setObjectFloatParameter(forwarder,sim.shapefloatparam_init_velocity_z,absoluteLinearVelocity[3])
end
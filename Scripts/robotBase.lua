-- This script controls the Youbot task. It is threaded. In next sections, there are a lot of function definitions
-- that ease the control of the robot. The arm of the robot is controlled in:
-- enjoy
-- 1. Forward kinematics mode ( setFkMode() )
-- 2. Inverse kinematics mode in position only ( setIkMode(false) )
-- 3. Inverse kinematics mode in position and orientation (to keep the gripper vertical) ( setIkMode(true) )
--
-- The inverse kinematics calculations are then automatically applied to the physics engine, since the joints are in
-- "hybrid IK mode" (see the joint properties dialog)
--
-- All inverse kinematics tasks are define in the IK properties dialog. There are 4 tasks:
--
-- "youBotUndamped_group": handles the inverse kinematics in position only. Resolution is not damped
-- "youBotDamped_group": handles the inverse kinematics in position only. Resolution is damped. Useful when the previous IK task didn't succeed, because out of reach for instance
-- "youBotUndamped_group": handles the inverse kinematics in position and orientation. Resolution is not damped
-- "youBotDamped_group": handles the inverse kinematics in position and orientation. Resolution is damped. Useful when the previous IK task didn't succeed, because out of reach for instance
--
-- Above 4 tasks are enabled/disabled as needed. This is done with the "sim.setExplicitHandling" function.

setIkMode=function(withOrientation)
    sim.setThreadAutomaticSwitch(false) -- Don't get interrupted for this function
    if (ikMode==false) then
        sim.setObjectPosition(gripperTarget,-1,sim.getObjectPosition(gripperTip,-1))
    end
    if (withOrientation) then
        sim.setExplicitHandling(ikWithOrientation1,0)
        sim.setExplicitHandling(ikWithOrientation2,0)
    else
        sim.setExplicitHandling(ik1,0)
        sim.setExplicitHandling(ik2,0)
    end
    for i=1,5,1 do
        sim.setJointMode(armJoints[i],sim.jointmode_ik,1)
    end
    ikMode=true
    sim.setThreadAutomaticSwitch(true)
end

setFkMode=function()
    sim.setThreadAutomaticSwitch(false) -- Don't get interrupted for this function
    sim.setExplicitHandling(ik1,1)
    sim.setExplicitHandling(ik2,1)
    sim.setExplicitHandling(ikWithOrientation1,1)
    sim.setExplicitHandling(ikWithOrientation2,1)

    for i=1,5,1 do
        sim.setJointMode(armJoints[i],sim.jointmode_force,0)
    end
    ikMode=false
    sim.setThreadAutomaticSwitch(true)
end

openGripper=function()
    sim.tubeWrite(gripperCommunicationTube,sim.packInt32Table({1}))
    sim.wait(0.8)
end

closeGripper=function()
    sim.tubeWrite(gripperCommunicationTube,sim.packInt32Table({0}))
    sim.wait(0.8)
end

setGripperTargetMovingWithVehicle=function()
    sim.setObjectParent(gripperTarget,vehicleReference,true)
end

setGripperTargetFixedToWorld=function()
    sim.setObjectParent(gripperTarget,-1,true)
end

waitToReachVehicleTargetPositionAndOrientation=function()
    repeat
        sim.switchThread() -- don't waste your time waiting!
        p1=sim.getObjectPosition(vehicleTarget,-1)
        p2=sim.getObjectPosition(vehicleReference,-1)
        p={p2[1]-p1[1],p2[2]-p1[2]}
        pError=math.sqrt(p[1]*p[1]+p[2]*p[2])
        oError=math.abs(sim.getObjectOrientation(vehicleReference,vehicleTarget)[3])
    until (pError<0.001)and(oError<0.1*math.pi/180) 
end

getPosAdjustedMatrixAndFacingAngle=function(posHandle)
    p2=sim.getObjectPosition(posHandle,-1)
    p1=sim.getObjectPosition(vehicleReference,-1)
    p={p2[1]-p1[1],p2[2]-p1[2],p2[3]-p1[3]}
    pl=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
    p[1]=p[1]/pl
    p[2]=p[2]/pl
    p[3]=p[3]/pl
    m=sim.getObjectMatrix(posHandle,-1)
    matchingScore=0
    for i=1,3,1 do
        v={m[0+i],m[4+i],m[8+i]}
        score=v[1]*p[1]+v[2]*p[2]+v[3]*p[3]
        if (math.abs(score)>matchingScore) then
            s=1
            if (score<0) then s=-1 end
            matchingScore=math.abs(score)
            bestMatch={v[1]*s,v[2]*s,v[3]*s}
        end
    end
    angle=math.atan2(bestMatch[2],bestMatch[1])
    m=sim.buildMatrix(p2,{0,0,angle})
    return m, angle-math.pi/2
end

pickupBoxFromPlace=function(posHandle,pickupConf)
    local m,angle=getPosAdjustedMatrixAndFacingAngle(posHandle)
    sim.setObjectPosition(vehicleTarget,-1,{m[4]-m[1]*dist1,m[8]-m[5]*dist1+0.08,0})
    sim.setObjectOrientation(vehicleTarget,-1,{0,0,angle})
    setFkMode()
    waitToReachVehicleTargetPositionAndOrientation()
    dispose=checkVisionSensor()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickupConf,nil)
    setIkMode(true)
    setGripperTargetFixedToWorld()
    local p=sim.getObjectPosition(gripperTarget,-1)
    p[1]=m[4]
    p[2]=m[8]
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    openGripper()
    p[3]=m[12]
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    closeGripper()
    p[3]=p[3]+0.1
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    setGripperTargetMovingWithVehicle()    
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickupConf,nil)
    --dispose code
    if(dispose) then
        disposeOfCube()
    end
    
    if(cacheSize==1 and not dispose) then
        dropToPlatform(pickupCfg,dropCfg,slot1)
    elseif(cacheSize==2 and not dispose) then
        dropToPlatform(pickupCfg,dropCfg,slot2)
    end
    

    
end

dropToPlatform=function(pickupCfg,dropCfg,slot)
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,dropCfg,nil)
    sim.rmlMoveToPosition(gripperTarget-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,dropCfg,nil,nil)
    setIkMode(false)
    local p=sim.getObjectPosition(slot,-1)
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    openGripper()
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,dropCfg,nil)
end

pickupFromPlatform=function(posHandle)
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,platformIntermediateDrop,nil)
    setIkMode(false)
    local p=sim.getObjectPosition(posHandle,-1)
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    closeGripper()
    setFkMode()
end

dropToPlace=function(placeHandle,shift,verticalPos,startConf,noVerticalArmForUpMovement)
    local m,angle=getPosAdjustedMatrixAndFacingAngle(placeHandle)
    m[4]=m[4]+m[2]*shift
    m[8]=m[8]+m[6]*shift
    sim.setObjectPosition(vehicleTarget,-1,{m[4]-m[1]*dist1+0.2,m[8]-m[5]*dist1,0})
    sim.setObjectOrientation(vehicleTarget,-1,{0,0,angle})
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,dropCfg,nil)
    waitToReachVehicleTargetPositionAndOrientation()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,startConf,nil)
    setIkMode(true)
    setGripperTargetFixedToWorld()
    local p=sim.getObjectPosition(gripperTarget,-1)
    p[1]=m[4]
    p[2]=m[8]
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    p[3]=verticalPos
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    openGripper()
    if (noVerticalArmForUpMovement) then
        setIkMode(false)
    end
    p[3]=p[3]+0.1
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    setGripperTargetMovingWithVehicle()    
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,dropCfg,nil)
end

goToCentre=function(posHandle,pickupCfg)
    local m,angle=getPosAdjustedMatrixAndFacingAngle(posHandle)
    sim.setObjectPosition(vehicleTarget,-1,{m[4]-m[1]*dist1,m[8]-m[5]*dist1+0.08,0})
    sim.setObjectOrientation(vehicleTarget,-1,{0,0,angle})
    setFkMode()
    sim.rmlMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,dropCfg,nil)
    waitToReachVehicleTargetPositionAndOrientation()
end

disposeOfCube=function()
    setIkMode(false)
    local p=sim.getObjectPosition(disposePos,-1)
    sim.rmlMoveToPosition(gripperTarget,-1,-1,nil,nil,ikSpeed,ikAccel,ikJerk,p,nil,nil)
    openGripper()
    setFkMode()
end

checkVisionSensor=function()
    local image=sim.getVisionSensorCharImage(visionSensor)
    local redPixel=image:byte(1)
	local greenPixel=image:byte(2)
	local bluePixel=image:byte(3)

    print(redPixel)
    print(greenPixel)
    print(bluePixel)
    
    if(redPixel <= 255 and redPixel >= 150) then
        table.insert(cache,"Red")
        cacheSize = cacheSize + 1
        print(cache)
        print(cacheSize)
        return false
    elseif(greenPixel <= 255 and greenPixel >= 150) then
        table.insert(cache,"Green")
        cacheSize = cacheSize + 1
        print(cache)
        print(cacheSize)
        return false
    elseif(bluePixel <= 255 and bluePixel >= 150) then
        table.insert(cache,"Blue")
        cacheSize = cacheSize + 1
        print(cache)
        print(cacheSize)
        return false
    else
        return true
    end
end

decideDropPos=function(index)
    if(cache[index]=="Red") then
        return redPlace
    elseif(cache[index]=="Blue") then
        return bluePlace
    elseif(cache[index]=="Green") then
        return greenPlace
    end
end

performSequence=function()
    goToCentre(centrePos,pickupCfg)
    
    while(cacheSize < 3) do
        pickupBoxFromPlace(pickupPos,pickupCfg)
    end

    for i=1,3 do
        goToCentre(centrePos,pickupCfg)
        dropPos=decideDropPos(4 - i)
        if(cacheSize == 2) then
            pickupFromPlatform(slot2)
        elseif(cacheSize == 1) then
            pickupFromPlatform(slot1)
        end 
        cacheSize = cacheSize - 1
        table.remove(cache,4 - i)
        print(cache)
        print(cacheSize)
        dropToPlace(dropPos,0,dropHeight,pickupCfg,false)
    end
end


function sysCall_threadmain()

    redColour = {1,0,0}
    greenColour = {0,1,0}
    blueColour = {0,0,1}
    trashColour = {0,0,0}

    gripperTarget=sim.getObjectHandle('youBot_gripperPositionTarget')
    gripperTip=sim.getObjectHandle('youBot_gripperPositionTip')

    vehicleReference=sim.getObjectHandle('youBot_vehicleReference')
    vehicleTarget=sim.getObjectHandle('youBot_vehicleTargetPosition')

    visionSensor=sim.getObjectHandle('visionSensor')
    cache={} --Empty table for storing colour blocks we have
    cacheSize=0
    --from vision sensor

    --redBox1=sim.getObjectHandle('redBox1')

    armJoints={-1,-1,-1,-1,-1}
    for i=0,4,1 do
        armJoints[i+1]=sim.getObjectHandle('youBotArmJoint'..i)
    end

    ik1=sim.getIkGroupHandle('youBotUndamped_group')
    ik2=sim.getIkGroupHandle('youBotDamped_group')
    ikWithOrientation1=sim.getIkGroupHandle('youBotPosAndOrientUndamped_group')
    ikWithOrientation2=sim.getIkGroupHandle('youBotPosAndOrientDamped_group')

    gripperCommunicationTube=sim.tubeOpen(0,'youBotGripperState'..sim.getNameSuffix(nil),1)

    redPlace=sim.getObjectHandle('redPos')
    bluePlace=sim.getObjectHandle('bluePos')
    greenPlace=sim.getObjectHandle('greenPos')

    pickupPos=sim.getObjectHandle('conveyerPos')
    disposePos=sim.getObjectHandle('conveyerTrashPos')
    centrePos=sim.getObjectHandle('centrePos')

    cubePos = {2.5750,-4.7528,0.15981}   
    box=sim.getObjectHandle('box')
    boxDummy=sim.getObjectHandle('boxPos')

    slot1=sim.getObjectHandle('slot1')
    slot2=sim.getObjectHandle('slot2')
    
    pickupCfg={180*math.pi/180,14.52*math.pi/180,50.27*math.pi/180,50.27*math.pi/180,0*math.pi/180}
    dropCfg={0,14.52*math.pi/180,50.27*math.pi/180,50.27*math.pi/180,0*math.pi/180}

    platformIntermediateDrop={0,16*math.pi/180,52*math.pi/180,73*math.pi/180,0*math.pi/180}

    dist1=0.2
    dropHeight=0.3

    ikSpeed={0.1,0.1,0.1,0.1}
    ikAccel={0.1,0.1,0.1,0.1}
    ikJerk={0.1,0.1,0.1,0.1}
    fkSpeed={1,1,1,1,1}
    fkAccel={0.6,0.6,0.6,0.6,0.6}
    fkJerk={1,1,1,1,1}

    conveyerDist=0.08
    shelfDist=0.2

    setGripperTargetMovingWithVehicle()
    setFkMode()
    openGripper()


    spawnCubes(12)
    for i=1,4 do
        performSequence()
    end

    goToCentre(centrePos,pickupCfg)


    sim.wait(2)
    
    sim.stopSimulation()
end

spawnCube=function()

    cubePos = {2.5750,-4.7528,0.15981}

    local copyCube=sim.copyPasteObjects({box,boxDummy},0)
    sim.setObjectPosition(copyCube[1],-1,cubePos)
    
    -- Randomly choose cube colour
    local ranNum = math.random(4)
    if (ranNum == 1) then
        colour = redColour
    elseif (ranNum == 2) then
        colour = greenColour
    elseif (ranNum == 3) then
        colour = blueColour
    else
        colour = trashColour
    end

    sim.setShapeColor(copyCube[1],nil,sim.colorcomponent_ambient_diffuse,colour)    
    
end

spawnCubes=function(amount)
    for i=1,amount do
        spawnCube()
        sim.wait(0.5)
    end
end
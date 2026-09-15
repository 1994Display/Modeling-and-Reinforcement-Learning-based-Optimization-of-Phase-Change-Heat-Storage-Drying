f = open(r'c:\Users\HTY\Desktop\gym\esp32_controller\esp32_controller.ino', 'r', encoding='utf-8')
lines = f.readlines()
f.close()

changes = 0

# 1. setDCMotor function: add railHomingDone protection
for i, line in enumerate(lines):
    if 'void setDCMotor(bool on) {' in line:
        # Find the existing check block
        target_idx = i + 2  # Line after the opening comment
        # Check if protection already exists
        protection_exists = False
        for j in range(i, min(i+20, len(lines))):
            if 'BLOCKED - rails not homed' in lines[j]:
                protection_exists = True
                break
        
        if not protection_exists:
            # Insert protection after the non-COLLECT check block
            # Find the line with 'return;' that closes the non-COLLECT check
            for j in range(i, min(i+10, len(lines))):
                if 'return;' in lines[j] and 'if (dcMotorOn)' in ''.join(lines[i:j]):
                    # Insert protection after this return line
                    protection = (
                        '  // 安全保护：导轨未回原点时禁止启动直流电机\n'
                        '  if (on && !railHomingDone) {\n'
                        '    if (dcMotorOn) setDCMotor(false);\n'
                        '    Serial.println("DC Motor: BLOCKED - rails not homed!");\n'
                        '    return;\n'
                        '  }\n'
                    )
                    lines.insert(j+1, protection)
                    changes += 1
                    print(f'1. Added railHomingDone protection in setDCMotor at line {j+2}')
                    break
        break

# 2. CS_HOMING completion: already has railHomingDone = true (verify)
for i, line in enumerate(lines):
    if 'railHomingDone = true;' in line:
        print(f'2. CS_HOMING completion railHomingDone=true at line {i+1} - OK')
        break

# 3. CS_DRY_HOMING completion: add railHomingDone = true
for i, line in enumerate(lines):
    if 'DRY: Home done -> Motor ON' in line:
        # Check if railHomingDone already exists nearby
        already_set = False
        for j in range(max(0, i-5), i):
            if 'railHomingDone = true' in lines[j]:
                already_set = True
                break
        if not already_set:
            # Find the 'if (inletDone && outletDone) {' line above this
            for j in range(i-1, max(i-15, 0), -1):
                if 'inletLastSW = false; outletLastSW = false;' in lines[j]:
                    lines.insert(j+1, '          railHomingDone = true;                  // 标记原点已回，允许启动直流电机\n')
                    changes += 1
                    print(f'3. Added railHomingDone=true at CS_DRY_HOMING completion, line {j+2}')
                    break
        else:
            print('3. CS_DRY_HOMING completion already has railHomingDone=true - OK')
        break

# 4. CS_PIPES_INSERT entry: add railHomingDone = false
for i, line in enumerate(lines):
    if 'COLLISION! Re-pressed - Motor STOP -> Insert pipes' in line:
        already_set = False
        for j in range(i+1, min(i+10, len(lines))):
            if 'railHomingDone = false' in lines[j]:
                already_set = True
                break
        if not already_set:
            for j in range(i+1, min(i+5, len(lines))):
                if 'collisionWasReleased = false;' in lines[j]:
                    lines.insert(j+1, '            railHomingDone = false;               // 导轨即将离开原点\n')
                    changes += 1
                    print(f'4. Added railHomingDone=false at CS_PIPES_INSERT entry, line {j+2}')
                    break
        else:
            print('4. CS_PIPES_INSERT entry already has railHomingDone=false - OK')
        break

# 5. CS_DRY_INSERT entry: add railHomingDone = false
for i, line in enumerate(lines):
    if 'DRY: Re-pressed! Motor STOP -> Insert pipes' in line:
        already_set = False
        for j in range(i+1, min(i+10, len(lines))):
            if 'railHomingDone = false' in lines[j]:
                already_set = True
                break
        if not already_set:
            for j in range(i+1, min(i+5, len(lines))):
                if 'collisionWasReleased = false;' in lines[j]:
                    lines.insert(j+1, '            railHomingDone = false;               // 导轨即将离开原点\n')
                    changes += 1
                    print(f'5. Added railHomingDone=false at CS_DRY_INSERT entry, line {j+2}')
                    break
        else:
            print('5. CS_DRY_INSERT entry already has railHomingDone=false - OK')
        break

f = open(r'c:\Users\HTY\Desktop\gym\esp32_controller\esp32_controller.ino', 'w', encoding='utf-8')
f.writelines(lines)
f.close()
print(f'\nDone - {changes} changes applied')

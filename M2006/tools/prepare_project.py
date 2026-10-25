"""Make an isolated HAL/Keil teaching project using locally installed ST drivers."""
from pathlib import Path
import shutil
import re
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
cube = Path('D:/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3')
template = root.parent/'16_云台调试/Output/Gimbal_Control/MDK-ARM/Gimbal_Control.uvprojx'
project = root/'MDK-ARM'
project.mkdir(exist_ok=True)
(root/'Drivers').mkdir(exist_ok=True)
for name in ['CMSIS', 'STM32F4xx_HAL_Driver']:
    shutil.copytree(cube/'Drivers'/name, root/'Drivers'/name, dirs_exist_ok=True)
conf = (cube/'Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_conf_template.h').read_text()
enabled = {'HAL_MODULE_ENABLED','HAL_CAN_MODULE_ENABLED','HAL_UART_MODULE_ENABLED',
           'HAL_RCC_MODULE_ENABLED','HAL_GPIO_MODULE_ENABLED','HAL_CORTEX_MODULE_ENABLED',
           'HAL_PWR_MODULE_ENABLED','HAL_FLASH_MODULE_ENABLED','HAL_DMA_MODULE_ENABLED'}
conf = re.sub(r'^#define (HAL_\w+_ENABLED)\s*$',
              lambda m: '#define '+m[1] if m[1] in enabled else '/* #define '+m[1]+' */',
              conf, flags=re.M)
(root/'firmware/stm32f4xx_hal_conf.h').write_text(conf)
startup = cube/'Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/arm/startup_stm32f405xx.s'
(project/'startup_stm32f405xx.s').write_text(startup.read_text().replace('0x00000400','0x00001000'))
shutil.copy2(cube/'Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c',root/'firmware')

tree = ET.parse(template)
target = tree.find('./Targets/Target')
target.find('TargetName').text = 'M2006_Debug'
for e in target.findall('.//OutputDirectory'): e.text = 'Objects\\'
for e in target.findall('.//OutputName'): e.text = 'M2006_Debug'
for e in target.findall('.//ListingPath'): e.text = 'Listings\\'
for e in target.findall('.//RunUserProg1')+target.findall('.//RunUserProg2'): e.text = '0'
includes = '../firmware;../Drivers/STM32F4xx_HAL_Driver/Inc;../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy;../Drivers/CMSIS/Device/ST/STM32F4xx/Include;../Drivers/CMSIS/Include'
for e in target.findall('.//VariousControls/IncludePath'): e.text = includes
for e in target.findall('.//Cads/VariousControls/Define'): e.text = 'USE_HAL_DRIVER,STM32F405xx,HSE_VALUE=25000000'
groups = target.find('Groups'); groups.clear()
hal_names = ['stm32f4xx_hal','stm32f4xx_hal_rcc','stm32f4xx_hal_rcc_ex',
             'stm32f4xx_hal_gpio','stm32f4xx_hal_cortex','stm32f4xx_hal_can',
             'stm32f4xx_hal_uart','stm32f4xx_hal_dma','stm32f4xx_hal_pwr',
             'stm32f4xx_hal_pwr_ex','stm32f4xx_hal_flash','stm32f4xx_hal_flash_ex']
for name, paths in [('Startup',['startup_stm32f405xx.s','../firmware/system_stm32f4xx.c']),
                    ('Motor Demo',['../firmware/main.c','../firmware/motor_demo.h']),
                    ('STM32 HAL',['../Drivers/STM32F4xx_HAL_Driver/Src/'+s+'.c' for s in hal_names])]:
    group=ET.SubElement(groups,'Group'); ET.SubElement(group,'GroupName').text=name
    files=ET.SubElement(group,'Files')
    for path in paths:
        f=ET.SubElement(files,'File'); ET.SubElement(f,'FileName').text=Path(path).name
        ET.SubElement(f,'FileType').text = '2' if path.endswith('.s') else ('5' if path.endswith('.h') else '1')
        ET.SubElement(f,'FilePath').text=path
ET.indent(tree,space='  '); tree.write(project/'M2006_Debug.uvprojx',encoding='utf-8',xml_declaration=True)

# Keep the working ST-LINK/SWD settings from a locally configured F405 project.
opt_source = root.parent/'14_板载IMU读取/Output/BMI088_Read/MDK-ARM/BMI088_Read.uvoptx'
opt = ET.parse(opt_source)
for group in opt.findall('Group'): opt.getroot().remove(group)
opt.find('./Target/TargetName').text = 'M2006_Debug'
for e in opt.findall('.//Breakpoint'):
    e.clear()
for e in opt.findall('.//DebugDescription/DbgClock'): e.text = '1000000'
for e in opt.findall('.//TargetDriverDllRegistry/SetRegEntry/Name'):
    if '-TC10000000' in (e.text or ''): e.text = e.text.replace('-TC10000000','-TC1000000')
ET.indent(opt,space='  ')
# Do not overwrite the user's later Watch/window preferences when rebuilding.
if not (project/'M2006_Debug.uvoptx').exists():
    opt.write(project/'M2006_Debug.uvoptx',encoding='utf-8',xml_declaration=True)
print('Project:',project/'M2006_Debug.uvprojx')

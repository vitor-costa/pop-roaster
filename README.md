Disclaimer: the initial code is not mine, but I don't know the ownership of everything

# Pop Roaster
Controlling a popcorn popper using Arduino to roast great coffee.

To get more information about the project and details how to build the roaster, read my personal blog on Medium (it's in PT-BR): https://medium.com/@relatos.cafe

## Release History

###  Version 0.3.0

 - Remove beta feature: heater limits by temperature. I figured out that it's better to control
 heater using only PID with decent configuration.

###  Version 0.2.1

 - Hardware protection by software:
   - Limiting maximum temperature at 220ºC.
   - Turn off heater if fan is set to zero.

### Version 0.2.0

 - New beta feature: heater limits by temperature for better automation (disabled by default).
 - Using PID controlled mode now with better response using different PID configuration and better thermocouple (faster response).
 - Major refactor and code sanitization.

### Version 0.1.0

First usable version.

- Allows control of fan and heat power by Roastlogger.
- Using PID controlled mode might still have some issues.
- Heater control system still causes visible RoR instability.

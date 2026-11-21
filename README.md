Simple Smoke Machine DMX Controller
===================================

- Two levels of isolation (relay + power isolation and optical coupler for DMX Line)
- Based on cheap ATTiny1614 micro-controller
- Only needs extra 5v power supply (it can be embed into CI and use smoke machine power)

The main goal is to be used to control a smoke machine, but the relay can be connected to other things

DMX Channel Programming
===

- Set all DMX Channel values to 0
- Set desired DMX Channel to 255
- Pull-up PA6 Pin
- If it succeeds smoke machine will be triggered
- Turn down the desired channel to stop smoke machine


#ifndef ControlDigIO_h
#define ControlDigIO_h

#include <Arduino.h>

//class ControlDigIO{
public:
	ControlDigIO();
	~ControlDigIO();
	void pindec();
	void ClearAllOutputs();
	void SetAllOutputs();
	byte SetValves_Default();
	byte SetValves_Pump();
	byte SetValves_CCRShot();
	byte SetValves_PumpAll();
	byte SetValves_FillBuffer();
	byte SetValves_FillBuffer_PumpCCR();
	byte SetValves_FlowGas();
	void ClearAllLEDs();
	void ClearButtonLEDs();
	void SetAllLEDs();
	void Pump_LEDSet();
	void Fill_LEDSet();
	void Flow_LEDSet();
	void Shot_LEDSet();
	byte PushButton_State();
private:
	void PumpValve(boolean control_state);
	void GasValve(boolean control_state);
	void BufferValve(boolean control_state);
	void Fault_LED(boolean control_state);
	void Shot_LED(boolean control_state);
	void Flow_LED(boolean control_state);
	void Fill_LED(boolean control_state);
	void Pump_LED(boolean control_state);
//};

#endif
/*package inverseKinematics;

import java.io.OutputStream;

import java.io.*;
import java.util.*;
import javax.comm.*;

public class SerialCommunicator {

	static Enumeration portList;
	static CommPortIdentifier portId;
	static SerialPort serialPort;
	static OutputStream outputStream;

	public SerialCommunicator() {
		init();
	}

	private void init() {
		portList = CommPortIdentifier.getPortIdentifiers();
		while (portList.hasMoreElements()) {
			portId = (CommPortIdentifier) portList.nextElement();
			if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				if (portId.getName().equals("COM1")) {
					try {
						serialPort = (SerialPort) portId.open("SimpleWriteApp", 2000);
					} catch (PortInUseException e) {
						e.printStackTrace();
					}
					try {
						outputStream = serialPort.getOutputStream();
					} catch (IOException e) {
						e.printStackTrace();
					}
					try {
						serialPort.setSerialPortParams(9600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1,
								SerialPort.PARITY_NONE);
					} catch (UnsupportedCommOperationException e) {
						e.printStackTrace();
					}
				}
			}
		}
	}

	public void sendMessage(String msg) {
		if (serialPort != null) {
			try {
				outputStream.write(msg.getBytes());
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
}
*/
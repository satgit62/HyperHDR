// HyperHDR local includes
#include <led-drivers/serial/DriverSerialAtmo.h>

DriverSerialAtmo::DriverSerialAtmo(const QJsonObject& deviceConfig)
	: ProviderSerial(deviceConfig)
{
}

LedDevice* DriverSerialAtmo::construct(const QJsonObject& deviceConfig)
{
	return new DriverSerialAtmo(deviceConfig);
}

bool DriverSerialAtmo::init(const QJsonObject& deviceConfig)
{
	bool isInitOK = false;

	// Initialise sub-class
	if (ProviderSerial::init(deviceConfig))
	{
		if (_ledCount != 5)
		{
			QString errortext = QString("%1 channels configured. This should always be 5!").arg(_ledCount);
			this->setInError(errortext);
			isInitOK = false;
		}
		else
		{
			_ledBuffer.resize(4 + 5 * 3); // 4-byte header, 5 RGB values
			_ledBuffer[0] = 0xFF;       // Startbyte
			_ledBuffer[1] = 0x00;       // StartChannel(Low)
			_ledBuffer[2] = 0x00;       // StartChannel(High)
			_ledBuffer[3] = 0x0F;       // Number of Databytes send (always! 15)

			isInitOK = true;
		}
	}
	return isInitOK;
}

int DriverSerialAtmo::write(const std::vector<ColorRgb>& ledValues)
{
	memcpy(4 + _ledBuffer.data(), ledValues.data(), _ledCount * sizeof(ColorRgb));
	return writeBytes(_ledBuffer.size(), _ledBuffer.data());
}

bool DriverSerialAtmo::isRegistered = hyperhdr::leds::REGISTER_LED_DEVICE("atmo", "leds_group_3_serial", DriverSerialAtmo::construct);

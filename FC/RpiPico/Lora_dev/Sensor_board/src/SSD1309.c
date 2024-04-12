#include "SSD1309.h"

/*!
	@brief  Constructor for I2C-interfaced OLED display.
	@param  DevAddr
			Device i2c address shifted one to the left.
	@param  width
			Display width.
	@param  height
			Display height.
	@param  i2c
			Pointer to an existing i2c instance.
	@return SSD1306 object.
*/
void SSD1309_create(SSD1309 *ssd1309, uint16_t DevAddr, uint8_t width, uint8_t height, i2c_inst_t *i2c, uint8_t resetPin)
{
	ssd1309 = (SSD1309 *)malloc(sizeof(SSD1309));
	if (ssd1309 == NULL)
	{
		while (1)
		{
			printf("Error: SSD1309_create: malloc failed\n");
			sleep_ms(1000);
		}
	}

	ssd1309->DevAddr = DevAddr;
	ssd1309->width = width;
	ssd1309->height = height;
	ssd1309->i2c = i2c;
	ssd1309->resetPin = resetPin;

	gpio_init(ssd1309->resetPin);
	gpio_set_dir(ssd1309->resetPin, GPIO_OUT);
	gpio_pull_up(ssd1309->resetPin);
	SSD1309_reset(ssd1309);

	ssd1309->buffer = (unsigned char *)malloc(width * height / 8);
	if (ssd1309->buffer == NULL)
	{
		while (1)
		{
			printf("Error: SSD1309_create: buffer malloc failed\n");
			sleep_ms(1000);
		}
	}
	SSD1309_displayON(ssd1309, 0);

	SSD1309_sendCommand(ssd1309, SSD1309_setLowCSAinPAM);
	SSD1309_sendCommand(ssd1309, SSD1309_setHighCSAinPAM);
	SSD1309_sendCommand(ssd1309, SSD1309_setMemoryAddressingMode);
	SSD1309_sendCommand(ssd1309, 0x00);

	SSD1309_setContrast(ssd1309, 0xFF);

	SSD1309_sendCommand(ssd1309, SSD1309_inversionOff);

	SSD1309_sendCommand(ssd1309, SSD1309_setMultiplexRatio);
	SSD1309_sendCommand(ssd1309, 0x3F);

	SSD1309_sendCommand(ssd1309, SSD1309_setDisplayOffset);
	SSD1309_sendCommand(ssd1309, 0x00);

	SSD1309_sendCommand(ssd1309, SSD1309_setDisplayClockDivideRatio);
	SSD1309_sendCommand(ssd1309, 0x80);

	SSD1309_sendCommand(ssd1309, SSD1309_setPreChargePeriod);
	SSD1309_sendCommand(ssd1309, 0x22);

	SSD1309_sendCommand(ssd1309, SSD1309_setCOMpinsHWconfig);
	SSD1309_sendCommand(ssd1309, 0x12);

	SSD1309_sendCommand(ssd1309, SSD1309_setVCOMHdeselectLevel);
	SSD1309_sendCommand(ssd1309, 0x40);

	SSD1309_sendCommand(ssd1309, SSD1309_followRAMcontent);

	SSD1309_displayON(ssd1309, 1);
	SSD1309_clear(ssd1309, BLACK);
	SSD1309_display(ssd1309, NULL);
	printf("SSD1309_create: Display created\n");
}

/*!
	@brief  Destructor for I2C-interfaced OLED display.
*/
void SSD1309_destroy(SSD1309 *ssd1309)
{
	if (ssd1309 != NULL)
	{
		free(ssd1309->buffer);
		free(ssd1309);
	}
}

/*!
 * @brief Reset display.
 *
 */
void SSD1309_reset(SSD1309 *ssd1309)
{
	gpio_put(ssd1309->resetPin, 0);
	sleep_us(5);
	gpio_put(ssd1309->resetPin, 1);
}

/*!
 * @brief Send command to display.
 *
 */
void SSD1309_sendCommand(SSD1309 *ssd1309, uint8_t command)
{
	uint8_t mess[2] = {0x00, command};
	i2c_write_blocking(ssd1309->i2c, ssd1309->DevAddr, mess, 2, false);
}

/*!
 * @brief Invert colors.
 *
 */
void SSD1309_invertColors(SSD1309 *ssd1309, uint8_t Invert)
{
	SSD1309_sendCommand(ssd1309, Invert ? SSD1309_inversionOn : SSD1309_inversionOff);
}

/*!
 * @brief Turn on display.
 * 0 – Turn OFF
 * 1 – Turn ON
 */
void SSD1309_displayON(SSD1309 *ssd1309, uint8_t On)
{
	SSD1309_sendCommand(ssd1309, On ? SSD1309_pwrOn : SSD1309_pwrOff);
}

/*!
 * @brief Set contrast.
 *
 */
void SSD1309_setContrast(SSD1309 *ssd1309, uint8_t Contrast)
{
	SSD1309_sendCommand(ssd1309, SSD1309_setContrastControl);
	SSD1309_sendCommand(ssd1309, Contrast);
}

/*!
 * @brief Draw pixel in the buffer.
 *
 */
void SSD1309_drawPixel(SSD1309 *ssd1309, int16_t x, int16_t y, colors Color)
{
	if ((x < 0) || (x >= ssd1309->width) || (y < 0) || (y >= ssd1309->height))
		return;

	switch (Color)
	{
	case WHITE:
		ssd1309->buffer[x + (y / 8) * ssd1309->width] |= (1 << (y & 7));
		break;
	case BLACK:
		ssd1309->buffer[x + (y / 8) * ssd1309->width] &= ~(1 << (y & 7));
		break;
	case INVERSE:
		ssd1309->buffer[x + (y / 8) * ssd1309->width] ^= (1 << (y & 7));
		break;
	}
}

/*!
 * @brief Clear the buffer.
 *
 */
void SSD1309_clear(SSD1309 *ssd1309, colors Color)
{
	switch (Color)
	{
	case WHITE:
		memset(ssd1309->buffer, 0xFF, (ssd1309->height * ssd1309->width / 8));
		break;
	case BLACK:
		memset(ssd1309->buffer, 0x00, (ssd1309->height * ssd1309->width / 8));
		break;
	}
}

/*!
 * @brief Send buffer to OLED.
 *
 */
void SSD1309_display(SSD1309 *ssd1309, unsigned char *data)
{
	if (data == NULL)
	{
		data = ssd1309->buffer;
		printf("SSD1309_display: Displaying default buffer\n");
	}
	else
	{
		printf("SSD1309_display: Displaying\n");
	}

	for (uint8_t i = 0; i < ssd1309->height / 8; i++)
	{
		SSD1309_sendCommand(ssd1309, 0xB0 + i);
		SSD1309_sendCommand(ssd1309, 0x00);
		SSD1309_sendCommand(ssd1309, 0x10);
		SSD1309_sendData(ssd1309, &data[ssd1309->width * i], ssd1309->width);
	}
}

/*!
 * @brief Send data to OLED GCRAM.
 *
 */
void SSD1309_sendData(SSD1309 *ssd1309, uint8_t *buffer, size_t buff_size)
{
	unsigned char mess[buff_size + 1];

	mess[0] = 0x40;
	memcpy(mess + 1, buffer, buff_size);

	i2c_write_blocking(ssd1309->i2c, ssd1309->DevAddr, mess, buff_size + 1, false);
}

/*!
 * @brief Return display height.
 *
 */
uint8_t SSD1309_getHeight(SSD1309 *ssd1309)
{
	return ssd1309->height;
}

/*!
 * @brief Return display width.
 *
 */
uint8_t SSD1309_getWidth(SSD1309 *ssd1309)
{
	return ssd1309->width;
}

/*!
 * @brief Rotate display.
 *
 */
void SSD1309_rotateDisplay(SSD1309 *ssd1309, uint8_t Rotate)
{
	if (Rotate)
	{
		SSD1309_sendCommand(ssd1309, SSD1309_setSegmentMapFlipped);
		SSD1309_sendCommand(ssd1309, SSD1309_setCOMoutputFlipped);
	}
	else
	{
		SSD1309_sendCommand(ssd1309, SSD1309_setSegmentMapReset);
		SSD1309_sendCommand(ssd1309, SSD1309_setCOMoutputNormal);
	}
}

/*!
	@brief  Scroll part of display horyzontaly to the right.
	@param  startColumn
			0-127 Start of the scroll area.
	@param  endColumn
			0-127 End of the scroll area. Must be larger than or equal to startColumn.
	@param  startRow
			0-7 Start of the scroll area.
	@param  endRow
			0-7 End of the scroll area. Must be larger than or equal to startRow.
	@param  interval
			Interval between each scroll step.
	@return SSD1309 object.
*/
void SSD1309_scrollHorizontalRight(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval)
{
	SSD1309_sendCommand(ssd1309, SSD1309_deactivateScroll);
	SSD1309_sendCommand(ssd1309, SSD1309_contHScrollSetupRight);
	SSD1309_sendCommand(ssd1309, 0x00);
	SSD1309_sendCommand(ssd1309, startRow);
	SSD1309_sendCommand(ssd1309, (uint8_t)interval);
	SSD1309_sendCommand(ssd1309, endRow);
	SSD1309_sendCommand(ssd1309, 0x00);
	SSD1309_sendCommand(ssd1309, startColumn);
	SSD1309_sendCommand(ssd1309, endColumn);
	SSD1309_sendCommand(ssd1309, SSD1309_activateScroll);
}

/*!
	@brief  Scroll part of display horyzontaly to the left.
	@param  startColumn
			0-127 Start of the scroll area.
	@param  endColumn
			0-127 End of the scroll area. Must be larger than or equal to startColumn.
	@param  startRow
			0-7 Start of the scroll area.
	@param  endRow
			0-7 End of the scroll area. Must be larger than or equal to startRow.
	@param  interval
			Interval between each scroll step.
	@return SSD1309 object.
*/
void SSD1309_scrollHorizontalLeft(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval)
{
	SSD1309_sendCommand(ssd1309, SSD1309_deactivateScroll);
	SSD1309_sendCommand(ssd1309, SSD1309_contHScrollSetupLeft);
	SSD1309_sendCommand(ssd1309, 0x00);
	SSD1309_sendCommand(ssd1309, startRow);
	SSD1309_sendCommand(ssd1309, (uint8_t)interval);
	SSD1309_sendCommand(ssd1309, endRow);
	SSD1309_sendCommand(ssd1309, 0x00);
	SSD1309_sendCommand(ssd1309, startColumn);
	SSD1309_sendCommand(ssd1309, endColumn);
	SSD1309_sendCommand(ssd1309, SSD1309_activateScroll);
}

/*!
	@brief  Scroll part of display verticaly and horyzontaly to the left.
	@param  startColumn
			0-127 Start of the scroll area.
	@param  endColumn
			0-127 End of the scroll area. Must be larger than or equal to startColumn.
	@param  startRow
			0-7 Start of the scroll area.
	@param  endRow
			0-7 End of the scroll area. Must be larger than or equal to startRow.
	@param  interval
			Interval between each scroll step.
	@param  horizontal
			1 - Turn on horizontal scroll, 0 - Turn off horizontal scroll.
	@param  scrollingOffset
			Vertical scrolling offset.
	@return SSD1309 object.
*/
void SSD1309_scrollVerticalyLeft(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval, uint8_t horizontal, uint8_t scrollingOffset)
{
	SSD1309_sendCommand(ssd1309, SSD1309_deactivateScroll);
	SSD1309_sendCommand(ssd1309, SSD1309_contVHScrollSetupLeft);
	SSD1309_sendCommand(ssd1309, horizontal);
	SSD1309_sendCommand(ssd1309, startRow);
	SSD1309_sendCommand(ssd1309, (uint8_t)interval);
	SSD1309_sendCommand(ssd1309, endRow);
	SSD1309_sendCommand(ssd1309, scrollingOffset);
	SSD1309_sendCommand(ssd1309, startColumn);
	SSD1309_sendCommand(ssd1309, endColumn);
	SSD1309_sendCommand(ssd1309, SSD1309_activateScroll);
}

/*!
	@brief  Scroll part of display verticaly and horyzontaly to the right.
	@param  startColumn
			0-127 Start of the scroll area.
	@param  endColumn
			0-127 End of the scroll area. Must be larger than or equal to startColumn.
	@param  startRow
			0-7 Start of the scroll area.
	@param  endRow
			0-7 End of the scroll area. Must be larger than or equal to startRow.
	@param  interval
			Interval between each scroll step.
	@param  horizontal
			1 - Turn on horizontal scroll, 0 - Turn off horizontal scroll.
	@param  scrollingOffset
			Vertical scrolling offset.
	@return SSD1309 object.
*/
void SSD1309_scrollVerticalyRight(SSD1309 *ssd1309, uint8_t startColumn, uint8_t endColumn, uint8_t startRow, uint8_t endRow, scrollInterval interval, uint8_t horizontal, uint8_t scrollingOffset)
{
	SSD1309_sendCommand(ssd1309, SSD1309_deactivateScroll);
	SSD1309_sendCommand(ssd1309, SSD1309_contVHScrollSetupRight);
	SSD1309_sendCommand(ssd1309, horizontal);
	SSD1309_sendCommand(ssd1309, startRow);
	SSD1309_sendCommand(ssd1309, (uint8_t)interval);
	SSD1309_sendCommand(ssd1309, endRow);
	SSD1309_sendCommand(ssd1309, scrollingOffset);
	SSD1309_sendCommand(ssd1309, startColumn);
	SSD1309_sendCommand(ssd1309, endColumn);
	SSD1309_sendCommand(ssd1309, SSD1309_activateScroll);
}
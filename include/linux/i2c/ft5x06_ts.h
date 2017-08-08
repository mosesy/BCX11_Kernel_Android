#ifndef __LINUX_FT5X0X_TS_H__
#define __LINUX_FT5X0X_TS_H__

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	10

#define PRESS_MAX	0xFF
#define FT_PRESS		0x7F

#define FT5X0X_NAME	"ft5x0x_ts"

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	4
#define FT_TOUCH_PRE_OFFSET	2
#define FT_TOUCH_SIZE_OFFSET	3
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5
#define FT_TOUCH_PRESSURE_H			7
#define FT_TOUCH_PRESSURE_L			8
#define FT_TOUCH_WIDTH_H			8
#define FT_TOUCH_WIDTH_L			9
#define FT_TOUCH_HEIGHT_H			9
#define FT_TOUCH_HEIGHT_L			0x0a

#define POINT_READ_BUF	(3 + 8 * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FT5x0x_REG_FW_VER		0xA6
#define FT5x0x_REG_POINT_RATE	0x88
#define FT5X0X_REG_THGROUP	0x80

int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
		    char *readbuf, int readlen);
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

/* The platform data for the Focaltech ft5x0x touchscreen driver */
struct ft5x0x_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int irq;
	unsigned int reset;
};

#endif


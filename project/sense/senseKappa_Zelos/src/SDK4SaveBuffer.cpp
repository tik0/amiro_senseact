#include <cstdio>
#include <cstring>

#include <SDK4/SDK4AcquireApi.h>
#include "SDK4SaveBuffer.h"
#include <time.h>



uint8_t clip(int32_t iValue)
{
	uint8_t bValue = 0;

	if (iValue < 0)
		bValue = 0;
	else if (iValue > 255)
		bValue = 255;
	else
		bValue = (uint8_t)(iValue);

	return bValue;
}

void yuv2bgr(int32_t y, int32_t u, int32_t v, uint8_t* dst)
{
	*(dst++) = clip((298 * y + 516 * u + 128) >> 8);         	// B
	*(dst++) = clip((298 * y - 100 * u - 208 * v + 128) >> 8);  // G
	*(dst++) = clip((298 * y + 409 * v + 128) >> 8);            // R
}

void YUV420toBmp(int width, int height, uint8_t* src, uint8_t*dst)
{
	//src:yu420 dest bgr line is DWORD align
	uint8_t* srcUV = src + height * width;

	uint8_t *uZeile = new uint8_t[width / 2];
	uint8_t *vZeile = new uint8_t[width / 2];

	for (int j = 0; j < height; j++)
	{
		uint8_t* line = dst + (height - 1 - j)*width * 3;
		for (int i = 0; i < width; i += 2)
		{
			if ((j % 2) == 0)//gerade Zeilen
			{
				uZeile[i / 2] = srcUV[width * j / 2 + i];
				vZeile[i / 2] = srcUV[width * j / 2 + i + 1];
			}// sonst werden die Werte aus der letzten Zeile genommen...

			int iY0 = (int)((src[width * j + i])) - 16;
			int iY1 = (int)((src[width * j + i + 1])) - 16;
			int iU = (int)(uZeile[i / 2]) - 128;
			int iV = (int)(vZeile[i / 2]) - 128;
			yuv2bgr(iY0, iU, iV, line);
			yuv2bgr(iY1, iU, iV, line + 3);
			line += 6;
		}
	}
	delete[] uZeile;
	delete[] vZeile;
}

void UYV422toBmp(int width, int height, uint8_t* src, uint8_t*dst)
{
	for (int j = 0; j < height; j++)
	{
		uint8_t* p = src + j*width * 2;
		uint8_t* q = dst + (height - 1 - j)*width * 3;
		for (int i = 0; i < width; i +=2)
		{
			int iY0 = (int)(p[1]) - 16;
			int iY1 = (int)(p[3]) - 16;
			int iU = (int)(p[0]) - 128;
			int iV = (int)(p[2]) - 128;
			yuv2bgr(iY0, iU, iV, q);
			yuv2bgr(iY1, iU, iV, q + 3);
			q += 6;
			p += 4;
		}
	}
}
void Y8toBmp(int width, int height, uint8_t* src, uint8_t*dst)
{
	for (int j = 0; j < height; j++)
	{
		uint8_t* p = src + j*width;
		uint8_t* q = dst + (height - 1 - j)*width * 3;
		for (int i = 0; i < width; i++)
		{
			q[0] = *p;
			q[1] = *p;
			q[2] = *p;
			q += 3;
			p++;
		}
	}
}

void Y14toBmp(int width, int height, uint8_t* src, uint8_t*dst)
{
	for (int j = 0; j < height; j++)
	{
		uint16_t* p = (uint16_t*)src + j*width ;
		uint8_t* q = dst + (height - 1 - j)*width * 3;
		for (int i = 0; i < width; i++)
		{
			q[0] = *p>>6;
			q[1] = *p>>6;
			q[2] = *p>>6;
			q += 3;
			p++;
		}
	}
}

void RGB888toBmp(int width, int height, uint8_t* src, uint8_t*dst)
{
	for (int j = 0; j < height; j++)
	{
		uint8_t* p = src + j*width*3;
		uint8_t* q = dst + (height - 1 - j)*width * 3;
		for (int i = 0; i < width; i++)
		{
			q[0] = p[2];
			q[1] = p[1];
			q[2] = p[0];
			q += 3;
			p += 3;
		}
	}
}
void BGR888toBmp(int width, int height, uint8_t* src, uint8_t*dst)
{
	for (int j = 0; j < height; j++)
	{
		uint8_t* p = src + j*width * 3;
		uint8_t* q = dst + (height - 1 - j)*width * 3;
		memcpy(q, p, width * 3);
	}
}
#pragma pack(1)

typedef struct
{
	char id[2];
	uint32_t filesize;
	uint16_t reserved[2];
	uint32_t headersize;
	uint32_t infoSize;
	int32_t width;
	int32_t height;
	uint16_t biPlanes;
	uint16_t bits;
	uint32_t biCompression;
	uint32_t biSizeImage;
	int32_t biXPelsPerMeter;
	int32_t biYPelsPerMeter;
	uint32_t biClrUsed;
	uint32_t biClrImportant;
} BMPHEAD;

SDK4_ERROR SDK4CreateDateTimeFilename(char* prefix, uint32_t frameid,char* filename,char* ext, size_t size)
{
	char name[256];
	time_t rawtime;
	struct tm * info;

	memset(filename, 0, 256);
	time(&rawtime);
	info = localtime(&rawtime);
	strftime(name, 256, "%Y%m%d-%H%M%S", info);
	if(prefix==NULL)
		sprintf(filename, "%s_%6.6d.%s", name, frameid, ext);
	else
		sprintf(filename, "%s_%s_%6.6d.%s",prefix, name, frameid, ext);
	return SDK4_ERR_SUCCESS;
}

SDK4_ERROR SDK4SaveBufferRaw(void* pBuffer, size_t length,  const char* filename)
{
	FILE* fp = fopen( filename, "wb");
	if (fp!=NULL)
	{
		fwrite(pBuffer,1, length, fp);
		fclose(fp);
		return SDK4_ERR_SUCCESS;
	}
	else
		return SDK4_ERR_INVALID_HANDLE;
}
SDK4_ERROR SDK4SaveBuffer(void* pBuffer, int32_t width, int32_t height, ENUM_PIXELFORMAT pixelformat, const char* filename)
{
	SDK4_ERROR err = SDK4_ERR_SUCCESS;
	if (width % 4 != 0)
		return SDK4_ERR_INVALID_PARAMETER;

	FILE* fp = fopen( filename, "wb");
	if (fp!=NULL)
	{
		uint8_t* src = (uint8_t*)pBuffer;
		uint8_t* dst = new uint8_t[width*height * 3];


		BMPHEAD bh;
		memset((char *)&bh, 0, sizeof(BMPHEAD)); /* sets everything to 0 */
		memcpy(bh.id, "BM", 2);
		bh.headersize = 54; // (for 24 bit images)
		bh.infoSize = 40;// (for 24 bit images)
		bh.width = width;
		bh.height = height;
		bh.biPlanes = 1;	// (for 24 bit images)
		bh.bits = 24;	// (for 24 bit images)
		bh.biCompression = 0; // (no compression)
		bh.filesize = bh.headersize + bh.height*bh.width * 3;//width%4==0!
		fwrite(&bh, 1, sizeof (bh), fp);

		switch (pixelformat)
		{
		case PIXELFORMAT_Y8:
			Y8toBmp(width, height, src, dst);
			break;
		case PIXELFORMAT_Y14:
		case PIXELFORMAT_D14:
			Y14toBmp(width, height, src, dst);
			break;
		case PIXELFORMAT_YUV420:
			YUV420toBmp(width, height, src, dst);
			break;
		case PIXELFORMAT_RGB888:
			RGB888toBmp(width, height, src, dst);
			break;
		default:
			return SDK4_ERR_INVALID_PARAMETER;
			break;
		}

		fwrite(dst, 1, width*height * 3, fp);
		delete[] dst;
		fclose(fp);
		return SDK4_ERR_SUCCESS;
	}
	else
		return SDK4_ERR_INVALID_HANDLE;

}

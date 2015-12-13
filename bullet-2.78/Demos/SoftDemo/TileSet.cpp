#include "TileSet.h"
#include <stdio.h>
#include <Windows.h>
#include <GL/GL.h>
#include "tgaloader.h"

TileSet::TileSet()
{
	m_sizeX	= 0;
	m_sizeY	= 0;
}

TileSet::~TileSet()
{
	clear();
}

void TileSet::loadFromFile(const char* filename)
{
	clear();

	FILE* f = fopen(filename, "rb");
	if(!f)
	{
		char str[256];
		sprintf_s(str, "FAILED TO OPEN FILE: %s\n", filename);
		OutputDebugStringA(str);
		return;
	}

	TGALoader	loader;

	int curTileIndex		= 0;
	Tile curTile;
	bool bReadingImageNames = true;
	char curLine[1024];
	bool bStartedNewTile	= false;
	while( fgets(curLine,1024,f) )
	{
		//printf("%s",curLine);
		if(curLine[0] == '#')
		{
			sscanf_s(curLine, "# %d %d\n", &m_sizeX, &m_sizeY);
			bReadingImageNames = false;
			continue;
		}
        
		if(bReadingImageNames)
		{
			char fileName[1024];
			strcpy(fileName, curLine);
			fileName[strlen(fileName)-2] = '\0';	// remove last '\n'
			//OutputDebugStringA(fileName);
			m_texIds.push_back((GLuint)-1);
			loader.loadOpenGLTexture(fileName, &m_texIds[m_texIds.size()-1], TGA_BILINEAR);
		}
		else
		{
			for(int i=0 ; curLine[i] != '\0' ; i++)
			{
				if(curLine[i] >= '1' && curLine[i] <= '9')
				{
					int index = (int)(curLine[i] - '1');
 					curTile.texId = m_texIds[index];
					bStartedNewTile	= true;
				}
				else if(curLine[i] >= 'a' && curLine[i] <= 'd')
				{
					curTile.orientation = (int)(curLine[i] - 'a');
					bStartedNewTile	= true;
				}
				else if(curLine[i] == ' ' || curLine[i] == '\n')
				{
					if(bStartedNewTile)
					{
						m_tiles.push_back(curTile);
						bStartedNewTile	= false;
					}
				}
			}
		}
	}
	//fscanf(f, "%s",)

	fclose(f);
}

void TileSet::clear()
{
	m_tiles.clear();
	m_sizeX = 0;
	m_sizeY = 0;

	if(m_texIds.size())
		glDeleteTextures(m_texIds.size(), &m_texIds[0]);
	m_texIds.clear();
}

void TileSet::draw()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glBindTexture(GL_TEXTURE_2D, m_texIds[0]);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();
	
	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisable(GL_TEXTURE_GEN_R);

	glEnable(GL_COLOR_MATERIAL);
	glColor3f(1.0f, 1.0f, 1.0f);

	static float gfFloorHeight = 0.1f;
	static float gfFloorSide	= 15.0f;
	
	for(int y=0 ; y < m_sizeY ; y++)
	{
		for(int x=0 ; x < m_sizeX ; x++)
		{
			const Tile& curTile	= m_tiles[x + y*m_sizeX];

			glBindTexture(GL_TEXTURE_2D, curTile.texId);
			glLoadIdentity();
			glRotatef( ((float)curTile.orientation) * 90.0f, 0.0f, 0.0f, 1.0f );	// rotate texture matrix

			glBegin(GL_QUADS);
			glTexCoord2f(0.0f, 0.0f);
			glVertex3f((x+0) * gfFloorSide, gfFloorHeight, (y+0) * gfFloorSide);

			glTexCoord2f(0.0f, 1.0f);
			glVertex3f((x+0) * gfFloorSide, gfFloorHeight, (y+1) * gfFloorSide);

			glTexCoord2f(1.0f, 1.0f);
			glVertex3f((x+1) * gfFloorSide, gfFloorHeight, (y+1) * gfFloorSide);

			glTexCoord2f(1.0f, 0.0f);
			glVertex3f((x+1) * gfFloorSide, gfFloorHeight, (y+0) * gfFloorSide);
			glEnd();
		}
	}

	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);

	glPopAttrib();
}

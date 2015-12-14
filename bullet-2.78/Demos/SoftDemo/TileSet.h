#ifndef TILESET_H
#define TILESET_H

#include <vector>
#include <Windows.h>
#include <GL/GL.h>

class TileSet
{
private:
	struct Tile
	{
		GLuint	texId;
		int		orientation;
		Tile() : texId(0), orientation(0) {}
	};
	std::vector<Tile>			m_tiles;
	int							m_sizeX;
	int							m_sizeY;
	std::vector<unsigned int>	m_texIds;
public:
	TileSet();
	~TileSet();
	void	loadFromFile(const char* filename);
	void	draw(int x_offset, int y_offset);
	void	clear();
};

#endif	// TILESET_H

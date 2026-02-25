#include <jps/search/jps.h>
#include <warthog/constants.h>
#include <warthog/domain/gridmap.h>

namespace jps::search
{

// computes the forced neighbours of a node.
// for a neighbour to be forced we must check that
// (a) the alt path from the parent is blocked and
// (b) the neighbour is not an obstacle.
// if the test succeeds, we set a bit to indicate
// the direction from the current node (tiles[4])
// to the forced neighbour.
//
// @return an integer value whose lower 8 bits indicate
// the directions of forced neighbours
//
// NB: the first 3 bits of the first 3 bytes of @param tiles represent
// a 3x3 block of nodes. the current node is at the centre of
// the block.
// its NW neighbour is bit 0
// its N neighbour is bit 1
// its NE neighbour is bit 2
// its W neighbour is bit 8
// ...
// its SE neighbour is bit 18
// There are optimisations below that use bitmasks in order
// to speed up forced neighbour computation.
// We use the revised forced neighbour rules described in
// [Harabor and Grastien, The JPS Pathfinding System, SoCS, 2012]
// These rules do not allow diagonal transitions that cut corners.
uint32_t
compute_forced(direction d, uint32_t tiles)
{
	// NB: to avoid branching statements, bit operations are
	// used below to determine which neighbours are traversable
	// and which are obstacles
	enum : uint32_t
	{
		NW = 0b00000000'00000000'00000001,
		N  = 0b00000000'00000000'00000010,
		NE = 0b00000000'00000000'00000100,
		W  = 0b00000000'00000001'00000000,
		X  = 0b00000000'00000010'00000000,
		E  = 0b00000000'00000100'00000000,
		SW = 0b00000001'00000000'00000000,
		S  = 0b00000010'00000000'00000000,
		SE = 0b00000100'00000000'00000000,
	};
	uint32_t ret = 0;
	switch(d)
	{
	case NORTH:
	{
		uint32_t branch_nw = ((tiles & 0b001'00000001'00000000) == 256);
		ret               |= (branch_nw << 3); // force west
		ret               |= (branch_nw << 5); // force northwest

		uint32_t branch_ne = ((tiles & 0b100'00000100'00000000) == 1024);
		ret               |= (branch_ne << 2); // force east
		ret               |= (branch_ne << 4); // force northeast
		break;
	}
	case SOUTH:
	{
		uint32_t branch_sw = ((tiles & 257) == 256);
		ret               |= (branch_sw << 3); // force west
		ret               |= (branch_sw << 7); // force southwest

		uint32_t branch_se = ((tiles & 1028) == 1024);
		ret               |= (branch_se << 2); // force east
		ret               |= (branch_se << 6); // force southeast
		break;
	}
	case EAST:
	{
		uint32_t branch_ne = ((tiles & 3) == 2);
		ret               |= branch_ne;        // force north
		ret               |= (branch_ne << 4); // force northeast

		uint32_t branch_se = ((tiles & 196608) == 131072);
		ret               |= (branch_se << 1); // force south
		ret               |= (branch_se << 6); // force southeast
		break;
	}
	case WEST:
	{
		uint32_t force_nw = ((tiles & 6) == 2);
		ret              |= force_nw;        // force north
		ret              |= (force_nw << 5); // force northwest

		uint32_t force_sw = ((tiles & 393216) == 131072);
		ret              |= (force_sw << 1); // force south
		ret              |= (force_sw << 7); // force southwest
		break;
	}
	default:
		break;
	}
	return ret;
}

// Computes the natural neighbours of a node.
//
// NB: the first 3 bits of the first 3 bytes of @param tiles represent
// a 3x3 block of nodes. the current node is at the centre of
// the block.
// its NW neighbour is bit 0
// its N neighbour is bit 1
// its NE neighbour is bit 2
// its W neighbour is bit 8
// ...
// its SE neighbour is bit 18
// There are optimisations below that use bitmasks in order
// to speed up forced neighbour computation.
uint32_t
compute_natural(direction d, uint32_t tiles)
{
	// In the shift operations below the constant values
	// correspond to bit offsets for direction
	uint32_t ret = 0;
	switch(d)
	{
	case NORTH:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		break;
	case SOUTH:
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		break;
	case EAST:
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		break;
	case WEST:
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		break;
	case NORTHWEST:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		ret |= (uint32_t)((tiles & 259) == 259) << 5;
		break;
	case NORTHEAST:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		ret |= (uint32_t)((tiles & 1030) == 1030) << 4;
		break;
	case SOUTHWEST:
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		ret |= (uint32_t)((tiles & 196864) == 196864) << 7;
		break;
	case SOUTHEAST:
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		ret |= (uint32_t)((tiles & 394240) == 394240) << 6;
		break;
	default:
		ret |= (uint32_t)((tiles & 2) == 2) << 0;
		ret |= (uint32_t)((tiles & 131072) == 131072) << 1;
		ret |= (uint32_t)((tiles & 1024) == 1024) << 2;
		ret |= (uint32_t)((tiles & 256) == 256) << 3;
		ret |= (uint32_t)((tiles & 259) == 259) << 5;
		ret |= (uint32_t)((tiles & 1030) == 1030) << 4;
		ret |= (uint32_t)((tiles & 196864) == 196864) << 7;
		ret |= (uint32_t)((tiles & 394240) == 394240) << 6;
		break;
	}
	return ret;
}

} // namespace jps::search

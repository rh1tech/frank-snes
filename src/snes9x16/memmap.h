/*****************************************************************************\
     Snes9x - Portable Super Nintendo Entertainment System (TM) emulator.
                This file is licensed under the Snes9x License.
   For further information, consult the LICENSE file in the root directory.
\*****************************************************************************/

#ifndef _MEMMAP_H_
#define _MEMMAP_H_

#define MEMMAP_BLOCK_SIZE	(0x1000)
#define MEMMAP_NUM_BLOCKS	(0x1000000 / MEMMAP_BLOCK_SIZE)
#define MEMMAP_SHIFT		(12)
#define MEMMAP_MASK			(MEMMAP_BLOCK_SIZE - 1)

#include <string>
#include <vector>
#include <cstdint>

struct CMemory
{
	/* Device port: upstream's 12 MB ceiling is a desktop number - the core
	   allocates it up front, and this board has 8 MB of PSRAM total, which
	   also has to hold WRAM, VRAM, the tile caches and the framebuffers.
	   6 MB is the largest SNES ROM this port has ever had to load (the
	   SuperFX case in load_rom_from_sd caps at exactly this), so it is the
	   real limit rather than an arbitrary trim. */
	enum
#ifdef FRANK_SNES_CPU_CORE_S9X16
	{ MAX_ROM_SIZE = 0x600000 };
#else
	{ MAX_ROM_SIZE = 0xC00000 };
#endif

	enum file_formats
	{ FILE_ZIP, FILE_JMA, FILE_DEFAULT };

	enum
	{ NOPE, YEAH, BIGFIRST, SMALLFIRST };

	enum
	{ MAP_TYPE_I_O, MAP_TYPE_ROM, MAP_TYPE_RAM };

	enum
	{
		MAP_CPU,
		MAP_PPU,
		MAP_LOROM_SRAM,
		MAP_LOROM_SRAM_B,
		MAP_HIROM_SRAM,
		MAP_DSP,
		MAP_SA1RAM,
		MAP_BWRAM,
		MAP_BWRAM_BITMAP,
		MAP_BWRAM_BITMAP2,
		MAP_SPC7110_ROM,
		MAP_SPC7110_DRAM,
		MAP_RONLY_SRAM,
		MAP_C4,
		MAP_OBC_RAM,
		MAP_SETA_DSP,
		MAP_SETA_RISC,
		MAP_BSX,
		MAP_NONE,
		MAP_LAST
	};

	uint8	NSRTHeader[32];
	int32	HeaderCount;

	/* Device port: WRAM/VRAM live in PSRAM, as in the 1.43 core this
	   replaces. 192 KB of the RP2350's 520 KB SRAM is not available to
	   hold them. Allocated in S9xInitMemory. */
	uint8	*RAM;
#ifdef FRANK_SNES_CPU_CORE_S9X16
	/* Device port: these were std::vector, which allocates through
	   operator new and would land in the RP2350's 520 KB of SRAM. ROMStorage
	   alone is 6 MB. Plain pointers, allocated in Init through the PSRAM
	   allocator - the same treatment the 1.43 core in this tree gives them. */
	uint8_t *ROMStorage;
	uint32_t ROMStorageSize;
#else
	std::vector<uint8_t> ROMStorage;
#endif
	uint8   *ROM;
#ifdef FRANK_SNES_CPU_CORE_S9X16
	uint8_t *SRAMStorage;
	uint32_t SRAMStorageSize;
#else
	std::vector<uint8_t> SRAMStorage;
#endif
	uint8	*SRAM;
	const size_t SRAM_SIZE = 0x80000;
	uint8	*VRAM;
	uint8	*FillRAM;
	uint8	*BWRAM;
	uint8	*C4RAM;
	uint8	*OBC1RAM;
	uint8	*BSRAM;
	uint8	*BIOSROM;

	uint8	*Map[MEMMAP_NUM_BLOCKS];
	uint8	*WriteMap[MEMMAP_NUM_BLOCKS];
	uint8	BlockIsRAM[MEMMAP_NUM_BLOCKS];
	uint8	BlockIsROM[MEMMAP_NUM_BLOCKS];
	uint8	ExtendedFormat;

	std::string ROMFilename;
	char	ROMName[ROM_NAME_LEN];
	char	ROMId[5];
	int32	CompanyId;
	uint8	ROMRegion;
	uint8	ROMSpeed;
	uint8	ROMType;
	uint8	ROMSize;
	uint32	ROMChecksum;
	uint32	ROMComplementChecksum;
	uint32	ROMCRC32;
	unsigned char ROMSHA256[32];
	int32	ROMFramesPerSecond;

	bool8	HiROM;
	bool8	LoROM;
	uint8	SRAMSize;
	uint32	SRAMMask;
	uint32	CalculatedSize;
	uint32	CalculatedChecksum;

	// ports can assign this to perform some custom action upon loading a ROM (such as adjusting controls)
	void	(*PostRomInitFunc) (void);

	bool8	Init (void);
	void	Deinit (void);

	int		ScoreHiROM (bool8, int32 romoff = 0);
	int		ScoreLoROM (bool8, int32 romoff = 0);
	int		First512BytesCountZeroes() const;
	uint32	HeaderRemove (uint32, uint8 *);
	uint32	FileLoader (uint8 *, const char *, uint32);
    bool8   LoadROMMem (const uint8 *, uint32, const char* optional_rom_filename = NULL);
	bool8	LoadROM (const char *);
    bool8	LoadROMInt (int32);
    void	RebindROMPointers (void);
#ifdef FRANK_SNES_CPU_CORE_S9X16
    /* Bytes actually available at ROM. The buffer is sized per-ROM here, so
       MAX_ROM_SIZE is a ceiling and NOT the allocation - using it to bound a
       memset writes megabytes past the end. */
    uint32	ROMBufferBytes (void) const
    { return ROMStorageSize > 0x8000u ? ROMStorageSize - 0x8000u : 0u; }
#endif
    bool8   LoadMultiCartMem (const uint8 *, uint32, const uint8 *, uint32, const uint8 *, uint32);
	bool8	LoadMultiCart (const char *, const char *);
    bool8	LoadMultiCartInt ();
	bool8	LoadSufamiTurbo ();
	bool8	LoadBSCart ();
	bool8	LoadGNEXT ();
	bool8	LoadSRAM (const char *);
	bool8	SaveSRAM (const char *);
	void	ClearSRAM (bool8 onlyNonSavedSRAM = 0);
	bool8	LoadSRTC (void);
	bool8	SaveSRTC (void);
	bool8	SaveMPAK (const char *);

	void	ParseSNESHeader (uint8 *);
	void	InitROM (void);

	uint32	map_mirror (uint32, uint32);
	void	map_lorom (uint32, uint32, uint32, uint32, uint32);
	void	map_hirom (uint32, uint32, uint32, uint32, uint32);
	void	map_lorom_offset (uint32, uint32, uint32, uint32, uint32, uint32);
	void	map_hirom_offset (uint32, uint32, uint32, uint32, uint32, uint32);
	void	map_space (uint32, uint32, uint32, uint32, uint8 *);
	void	map_index (uint32, uint32, uint32, uint32, int, int);
	void	map_System (void);
	void	map_WRAM (void);
	void	map_LoROMSRAM (void);
	void	map_HiROMSRAM (void);
	void	map_DSP (void);
	void	map_C4 (void);
	void	map_OBC1 (void);
	void	map_SetaRISC (void);
	void	map_SetaDSP (void);
	void	map_WriteProtectROM (void);
	void	Map_Initialize (void);
	void	Map_LoROMMap (void);
	void	Map_NoMAD1LoROMMap (void);
	void	Map_JumboLoROMMap (void);
	void	Map_ROM24MBSLoROMMap (void);
	void	Map_SRAM512KLoROMMap (void);
	void	Map_SufamiTurboLoROMMap (void);
	void	Map_SufamiTurboPseudoLoROMMap (void);
	void	Map_SuperFXLoROMMap (void);
	void	Map_SetaDSPLoROMMap (void);
	void	Map_SDD1LoROMMap (void);
	void	Map_SA1LoROMMap (void);
	void	Map_BSSA1LoROMMap (void);
	void	Map_HiROMMap (void);
	void	Map_ExtendedHiROMMap (void);
	void	Map_SPC7110HiROMMap (void);
	void	Map_BSCartLoROMMap(uint8);
	void	Map_BSCartHiROMMap(void);

	uint16	checksum_calc_sum (uint8 *, uint32);
	uint16	checksum_mirror_sum (uint8 *, uint32 &, uint32 mask = 0x800000);
	void	Checksum_Calculate (void);

	bool8	match_na (const char *);
	bool8	match_nn (const char *);
	bool8	match_nc (const char *);
	bool8	match_id (const char *);
	void	ApplyROMFixes (void);
    std::string SafeString(std::string s, bool allow_jis = false);
	void	CheckForAnyPatch (const char *, bool8, int32 &);

	void	MakeRomInfoText (char *);
	std::string GetMultilineROMInfo();

	const char *	MapType (void);
	const char *	StaticRAMSize (void);
	const char *	Size (void);
	const char *	Revision (void);
	const char *	KartContents (void);
	const char *	Country (void);
	const char *	PublishingCompany (void);
};

struct SMulti
{
	int		cartType;
	int32	cartSizeA, cartSizeB;
	int32	sramSizeA, sramSizeB;
	uint32	sramMaskA, sramMaskB;
	uint32	cartOffsetA, cartOffsetB;
	uint8	*sramA, *sramB;
	char	fileNameA[PATH_MAX + 1], fileNameB[PATH_MAX + 1];
};

extern CMemory	Memory;
extern SMulti	Multi;

inline bool S9xInterlaceField()
{
	return (Memory.FillRAM[0x213F] & 0x80) >> 7;
}

void S9xAutoSaveSRAM (void);
bool8 LoadZip(const char *, uint32 *, uint8 *);

enum s9xwrap_t
{
	WRAP_NONE,
	WRAP_BANK,
	WRAP_PAGE
};

enum s9xwriteorder_t
{
	WRITE_01,
	WRITE_10
};

#include "getset.h"

#endif

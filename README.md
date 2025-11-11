# Disk Scheduling Simulator

This repository contains both Python and C++ implementations of a disk scheduling simulator for educational purposes.

## Files

- **disk.py** - Original Python implementation (requires tkinter for graphics)
- **disk.cpp** - C++ implementation (console-only)
- **Makefile** - Build file for C++ version

## Building the C++ Version

```bash
make
```

This will compile `disk.cpp` into an executable called `disk`.

## Usage

### C++ Version

Run the simulator with default settings:
```bash
./disk -c
```

### Command-Line Options

- `-s, --seed <N>` - Random seed (default: 0)
- `-a, --addr <LIST>` - Request list (comma-separated) or -1 for random
- `-A, --addrDesc <DESC>` - Address descriptor: numRequests,maxRequest,minRequest (default: "5,-1,0")
- `-S, --seekSpeed <N>` - Speed of seek (default: 1)
- `-R, --rotSpeed <N>` - Speed of rotation (default: 1)
- `-p, --policy <POLICY>` - Scheduling policy: FIFO, SSTF, SATF, BSATF (default: FIFO)
- `-w, --schedWindow <N>` - Scheduling window size, -1 for all (default: -1)
- `-o, --skewOffset <N>` - Skew offset in blocks (default: 0)
- `-z, --zoning <ZONES>` - Angles between blocks on outer,middle,inner tracks (default: "30,30,30")
- `-c, --compute` - Show computed results
- `-l, --lateAddr <LIST>` - Late arriving requests
- `-L, --lateAddrDesc <DESC>` - Late address descriptor (default: "0,-1,0")

### Examples

Test FIFO scheduling:
```bash
./disk -c -s 0 -p FIFO
```

Test SATF (Shortest Access Time First) scheduling:
```bash
./disk -c -s 0 -p SATF
```

Test SSTF (Shortest Seek Time First) scheduling:
```bash
./disk -c -s 0 -p SSTF
```

Specify custom requests:
```bash
./disk -c -a 10,15,20,5,18
```

Generate 10 random requests:
```bash
./disk -c -A 10,-1,0
```

## Scheduling Policies

- **FIFO** - First In First Out (processes requests in order)
- **SSTF** - Shortest Seek Time First (minimizes seek distance)
- **SATF** - Shortest Access Time First (minimizes total access time)
- **BSATF** - Bounded SATF (SATF with fairness window)

## Output

The simulator outputs:
- **Seek time** - Time spent moving the disk arm between tracks
- **Rotate time** - Time spent waiting for the platter to rotate
- **Transfer time** - Time spent reading/writing data
- **Total time** - Sum of all times

Example output:
```
Block:   8  Seek:  0  Rotate: 45  Transfer: 30  Total:  75
Block:  11  Seek:  0  Rotate: 60  Transfer: 30  Total:  90
Block:   2  Seek:  0  Rotate: 60  Transfer: 30  Total:  90
Block:  15  Seek: 80  Rotate:280  Transfer: 30  Total: 390
Block:  18  Seek:  0  Rotate: 60  Transfer: 30  Total:  90

TOTALS      Seek: 80  Rotate:505  Transfer:150  Total: 735
```

## Differences from Python Version

- Graphics not supported (Python version uses Tkinter)
- Console output only
- Same simulation logic and algorithms
- Identical numerical results

## Cleaning Up

To remove compiled files:
```bash
make clean
```

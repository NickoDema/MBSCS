/* map_keeper.cpp
 *
 *  Created on: 20.04.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#include <mapper.h>

Mapper::Map_keeper::Map_keeper(int size)
{

		for(int i = 0; i < size; i++) {
			for(int j = 0; j < size; j++) {
				map_[i][j] = -1;
			}
		}
		x_error = 0;
		y_error = 0;
}

void Mapper::Map_keeper::move (int cells, char dir)
{
	std::cout << dir << " " << cells << " dir and cells in move function" << std::endl;	//--
	if (cells > CELL_N) cells = CELL_N;
	if (cells <= 0) return;
		if (dir == 'f')						//forward
		{
			for(int i = 0; i < CELL_N; i++) {
				for(int j = 0; j < CELL_N-cells; j++) {
					map_[i][j] = map_[i][j+cells];
				}
			}

			for(int i = 0; i < CELL_N; i++) {
				for(int j = CELL_N - 1; j > CELL_N - cells; j--) {
					map_[i][j] = -1;
				}
			}
		
		}
		else if (dir == 'b')				//backward
		{
			for(int i = 0; i < CELL_N; i++) {
				for(int j = CELL_N - 1; j >= cells; j--) {
					map_[i][j] = map_[i][j-cells];
				}
			}
			for(int i = 0; i < CELL_N; i++) {
				for(int j = 0; j < cells; j++) {
					map_[i][j] = -1;
				}
			}
		
		}
		else if (dir == 'l')				//to the left
		{
			for(int i = 0; i < CELL_N - cells; i++) {
				for(int j = 0; j < CELL_N; j++) {
					map_[i][j] = map_[i+cells][j];
				}
			}
			for(int i = CELL_N - 1; i >= CELL_N - cells; i--) {
				for(int j = 0; j < CELL_N - 1; j++) {
					map_[j][i] = -1;
				}
			}
		}
		else if (dir == 'r')				//to the right
		{
			for(int i = CELL_N - 1; i >= cells; i--) {
				for(int j = 0; j < CELL_N; j++) {
					map_[i][j] = map_[i-cells][j];
				}
			}
			for(int i = 0; i < cells; i++) {
				for(int j = 0; j < CELL_N - 1; j++) {
					map_[j][i] = -1;
				}
			}
		
		}
}
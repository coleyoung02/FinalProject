#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <string>
#undef main
	

#define fluidResolution 64
#define windowSize 1024
#define gravityConstant 9.8
#define divergenceMult 1.085
#define divergenceCap 20000
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::steady_clock::time_point stamp;

void rerender(SDL_Renderer* r, float densities[fluidResolution][fluidResolution], float x[fluidResolution][fluidResolution], float y[fluidResolution][fluidResolution]) {
	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			int d = floor(densities[j][i] * 255);
			int vx = floor(x[j][i] * 160);
			int vy = floor(y[j][i]);
			SDL_SetRenderDrawColor(r, d, 0, 0, 255);
			SDL_RenderDrawPoint(r, j, i);
		}
	}
	SDL_RenderPresent(r);

}

float getTimeSince(const stamp start) {
	stamp present = Time::now();
	return (std::chrono::duration<float>(present - start).count());
}

void applyGravity(float v_y[fluidResolution][fluidResolution], const float& deltaT, float density[fluidResolution][fluidResolution]) {
	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			v_y[i][j] += deltaT * gravityConstant;
		}
	}
}

void removeDivergence(float v_x[fluidResolution][fluidResolution], 
	float v_y[fluidResolution][fluidResolution], 
	bool obstacles[fluidResolution + 2][fluidResolution + 2]) {

	float div;
	int freeCount = 0;
	for (int _ = 0; _ < fluidResolution; ++_) {
		for (int i = 0; i < fluidResolution; ++i) {
			for (int j = 0; j < fluidResolution; ++j) {

				div = 0.0;
				freeCount = 4;
				if (i + 1 < fluidResolution) {
					div += v_x[i + 1][j];
				}
				else if (i == 0) {
					--freeCount;
				}
				else {
					--freeCount;
				}
				if (j + 1 < fluidResolution) {
					div += v_y[i][j + 1];
				}
				else if (j == 0) {
					--freeCount;
				}
				else {
					--freeCount;
				}
				div -= v_x[i][j];
				div -= v_y[i][j];
				div *= divergenceMult;
				float k = v_x[i][j];
				float divAdjust = freeCount * div / 4.0;
				if (divAdjust > divergenceCap) {
					divAdjust = divergenceCap;
				}
				if (divAdjust < -divergenceCap) {
					divAdjust = -divergenceCap;
				}
				if (i != 0 && j != 0) {
					v_x[i][j] += divAdjust;
				}
				if (i + 1 < fluidResolution)
					v_x[i + 1][j] -= divAdjust;
				if (j != 0) {
					v_y[i][j] += divAdjust;
				}
				if (j + 1 < fluidResolution)
					v_y[i][j + 1] -= divAdjust;
			}
		}
	}
}

float getWeightedValue(float x, 
	float y, 
	const float arr[fluidResolution][fluidResolution],
	bool velocityMode=false,
	std::string callContext = "") {
	if (x < 0) {
		if (velocityMode) {
			return 0;
		}
		x = 0;
	}
	else if (x >= fluidResolution) {

		if (velocityMode) {
			return 0;
		}
		x = fluidResolution - 1;
	}
	if (y < 0) {

		if (velocityMode) {
			return 0;
		}
		y = 0;
	}
	else if (y >= fluidResolution) {

		if (velocityMode) {
			return 0;
		}
		y = fluidResolution - 1;
	}

	float d_x = x - (int)x;
	float d_y = y - (int)y;

	if ((int)y >= fluidResolution - 1 && (int)x >= fluidResolution - 1) {
		return arr[(int)x][(int)y];
	}
	else if ((int)y >= fluidResolution - 1) {
		return arr[(int)x][(int)y] * (1 - d_x) +
			arr[(int)x + 1][(int)y] * (d_x);
	}
	else if ((int)x >= fluidResolution - 1) {
		return arr[(int)x][(int)y] * (1 - d_y) +
			arr[(int)x][(int)y + 1] * (d_y);
	}
	return arr[(int)x][(int)y] * (1 - d_x) * (1 - d_y) +
		arr[(int)x + 1][(int)y] * (d_x) * (1 - d_y) +
		arr[(int)x][(int)y + 1] * (1 - d_x) * (d_y)+
		arr[(int)x + 1][(int)y + 1] * (d_x) * (d_y);
}

void advect(float v_x[fluidResolution][fluidResolution],
	float v_y[fluidResolution][fluidResolution],
	float delta_time) {


	float current_v_x;
	float current_v_y;
	float new_v_x[fluidResolution][fluidResolution];
	float new_v_y[fluidResolution][fluidResolution];

	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			current_v_x = v_x[i][j];
			current_v_y = getWeightedValue(i + .5, j + .5, v_y, true, "1");
			new_v_x[i][j] = getWeightedValue(i - current_v_x * delta_time, j - current_v_y * delta_time, v_x, true, 
				"2 " + std::to_string(delta_time) + " " + std::to_string(current_v_y) + " " + std::to_string(i) + " " + std::to_string(j));


			current_v_x = getWeightedValue(i - .5, j - .5, v_y, true, "3");
			current_v_y = v_y[i][j];
			new_v_y[i][j] = getWeightedValue(i - current_v_x * delta_time, j - current_v_y * delta_time, v_y, true, "4");
		}
	}

	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			v_x[i][j] = new_v_x[i][j];
			v_y[i][j] = new_v_y[i][j];
		}
	}

}

void advectDensities(float v_x[fluidResolution][fluidResolution],
	float v_y[fluidResolution][fluidResolution],
	float densities[fluidResolution][fluidResolution],
	float delta_time) {

	float v_x_point;
	float v_y_point;
	float new_densities[fluidResolution][fluidResolution];
	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			v_x_point = (v_x[i][j] + v_x[std::min(i + 1, fluidResolution - 1)][j]) * .5;
			v_y_point = (v_y[i][j] + v_y[i][std::min(j + 1, fluidResolution - 1)]) * .5;
			new_densities[i][j] = getWeightedValue(i - v_x_point * delta_time, j - v_y_point * delta_time, densities);
		}
	}
	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			densities[i][j] = new_densities[i][j];
		}
	}
}

void gridBased() {

	float densities[fluidResolution][fluidResolution];
	bool obstacles[(fluidResolution + 2)][(fluidResolution + 2)];

	//initialize walls
	for (int i = 0; i < fluidResolution + 2; ++i) {
		for (int j = 0; j < fluidResolution + 2; ++j) {
			if (i == 0 || j == 0 || i == fluidResolution + 1 || j == fluidResolution + 1) {
				obstacles[i][j] = true;
			}
			else {
				obstacles[i][j] = false;
			}
		}
	}

	float v_x[fluidResolution][fluidResolution];
	float v_y[fluidResolution][fluidResolution];

	//initialize circle of fluid
	float centrality;
	float x;
	float y;
	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			x = ((float)i) / fluidResolution - .5;
			y = ((float)j) / fluidResolution - .5;
			centrality = 1.0 - sqrt(x*x + y*y);
			if (centrality > .9) {
				densities[i][j] = 1;
			}
			densities[i][j] = 0;
			v_x[i][j] = 0;
			v_y[i][j] = 0;
		}
	}

	SDL_Window* window = nullptr;
	SDL_Renderer* renderer = nullptr;

	SDL_Init(SDL_INIT_VIDEO);
	SDL_CreateWindowAndRenderer(windowSize, windowSize, 0, &window, &renderer);
	SDL_RenderSetLogicalSize(renderer, fluidResolution, fluidResolution);

	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);
	SDL_RenderPresent(renderer);

	SDL_Delay(1000);
	stamp t0 = Time::now();
	stamp last = Time::now();
	float t;

	//simulation loop
	while ((t = getTimeSince(t0)) < 20) {
		while (getTimeSince(last) < .025) {
			SDL_Delay(1);
		}
		float deltaT = getTimeSince(last);
		last = Time::now();

		applyGravity(v_y, deltaT, densities);
		//removeDivergence(v_x, v_y, obstacles);
		advect(v_x, v_y, deltaT);
		rerender(renderer, densities, v_x, v_y);
		advectDensities(v_x, v_y, densities, deltaT);
		for (int i = 0; i < fluidResolution; ++i) {
			for (int j = 0; j < fluidResolution; ++j) {
				x = ((float)i) / fluidResolution - (getTimeSince(t0) / 4 - (int)(getTimeSince(t0) / 4) );
				y = ((float)j) / fluidResolution - .5;
				centrality = 1.0 - sqrt(x * x + y * y);
				if (centrality > .9) {
					densities[i][j] = 1;
				}
			}
		}
	}
	SDL_Delay(5000);

}

void test(int x[4][4]) {
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			x[i][j] = i + j;
		}
	}
}

int main() {
	gridBased();
	return 0;
}


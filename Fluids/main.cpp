#include <SDL2/SDL.h>
#undef main


#define fluidResolution 128
#define windowSize 512

void rerender(SDL_Renderer* r, float* densities) {
	SDL_RenderClear(r);
	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			int b = densities[i * fluidResolution + j] * 255;
			SDL_SetRenderDrawColor(r, b, b, b, 255);
			for (int a = 0; a < windowSize / fluidResolution; ++a) {
				for (int b = 0; b < windowSize / fluidResolution; ++b) {
					SDL_RenderDrawPoint(r, j * windowSize / fluidResolution + b, i * windowSize / fluidResolution + a);
				}
			}
		}
	}
	SDL_RenderPresent(r);

}

int main() {

	float densities[fluidResolution * fluidResolution];
	for (int i = 0; i < fluidResolution; ++i) {
		for (int j = 0; j < fluidResolution; ++j) {
			densities[i * fluidResolution + j] = ((float)i) / fluidResolution + ((float)j) / fluidResolution / fluidResolution;
		}
	}

	SDL_Window* window = nullptr;
	SDL_Renderer* renderer = nullptr;

	SDL_Init(SDL_INIT_VIDEO);
	SDL_CreateWindowAndRenderer(windowSize, windowSize, 0, &window, &renderer);
	
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);

	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	SDL_RenderDrawPoint(renderer, 512 / 2, 512 / 2);

	SDL_RenderPresent(renderer);
	SDL_Delay(1000);
	rerender(renderer, densities);
	SDL_Delay(5000);

}


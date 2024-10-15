// A simple scope timer for measuring execution times.
// Store a sliding window of measurements and return the max/mean.
export class TicToc {
  private durations: Array<number> = [];
  private capacity: number;

  constructor(capacity?: number) {
    this.capacity = capacity || 1000;
  }

  public measureSpan<ReturnType>(scope: () => ReturnType): ReturnType {
    const start = performance.now();
    const result = scope();
    const end = performance.now();
    this.durations.push(end - start);
    if (this.durations.length > this.capacity) {
      this.durations.shift();
    }
    return result;
  }

  // Return max and mean execution time in milliseconds.
  public computeStats(): { max: number; mean: number } {
    if (!this.durations.length) {
      return { max: 0, mean: 0 };
    }
    const averageDuration = this.durations.reduce((a, b) => a + b) / this.durations.length;
    const maxDuration = Math.max(...this.durations);
    return { max: maxDuration, mean: averageDuration };
  }
}

import {
  massLocationsFromState,
  Point,
  ScaleAndTranslate,
  SingleCartPoleState,
  SingleCartPoleParams
} from './interfaces';

export class MouseInteraction {
  public massIndex: number;
  public incidentAngle: number;
  public clicked: boolean;

  constructor(massIndex: number, incidentAngle: number, clicked: boolean) {
    this.massIndex = massIndex;
    this.incidentAngle = incidentAngle;
    this.clicked = clicked;
  }
}

function indexOfSmallest<T>(a: Array<T>) {
  console.assert(a.length > 0);
  var lowest = 0;
  for (var i = 1; i < a.length; i++) {
    if (a[i] < a[lowest]) lowest = i;
  }
  return lowest;
}

// Handle user interaction with the cart-pole system via mouse events.
export class MouseHandler {
  private mousePosition: Point | null = null;
  private hasPendingClick: boolean = false;
  private activeIndex: number | null = null;

  constructor() {
    const canvas: HTMLCanvasElement = document.getElementById('canvas') as HTMLCanvasElement;

    canvas.addEventListener('mouseenter', (e) => {
      this.mouseEnter(e);
    });
    canvas.addEventListener('mouseleave', (e) => {
      this.mouseLeave(e);
    });
    canvas.addEventListener('mousemove', (e) => {
      this.mouseMove(e);
    });
    canvas.addEventListener('mousedown', (e) => {
      this.mouseDown(e);
    });
    canvas.addEventListener('mouseup', (e) => {
      this.mouseUp(e);
    });
  }

  // Given the current state of the system, check what interaction to execute.
  public determineInteraction(
    state: SingleCartPoleState,
    dynamicsParams: SingleCartPoleParams,
    pixelsFromMetric: ScaleAndTranslate
  ): MouseInteraction | null {
    if (this.mousePosition == null) {
      return null;
    }
    const { x: mx, y: my } = this.mousePosition;

    // Figure out which mass the mouse is closest to:
    const locations = massLocationsFromState(state, dynamicsParams);
    const scaledLocations = locations.map((p) => pixelsFromMetric.transform(p));
    const selectedIndex = this.maybeUpdateSelection(scaledLocations);

    // Convert to an incident angle.
    const angle = Math.atan2(
      my - scaledLocations[selectedIndex].y,
      mx - scaledLocations[selectedIndex].x
    );
    return new MouseInteraction(selectedIndex, angle, selectedIndex === this.activeIndex);
  }

  private maybeUpdateSelection(scaledLocations: Array<Point>) {
    if (this.activeIndex == null) {
      // No point is active, figure out the closest one:
      const { x: mx, y: my } = this.mousePosition;
      const distances = scaledLocations.map((p) =>
        Math.sqrt(Math.pow(mx - p.x, 2) + Math.pow(my - p.y, 2))
      );
      const minIndex = indexOfSmallest(distances);
      if (this.hasPendingClick) {
        this.activeIndex = minIndex;
      }
      return minIndex;
    }
    return this.activeIndex;
  }

  private mouseEnter(event: MouseEvent) {}
  private mouseLeave(event: MouseEvent) {
    this.mousePosition = null;
    this.hasPendingClick = false;
    this.activeIndex = null;
  }
  private mouseMove(event: MouseEvent) {
    this.mousePosition = { x: event.offsetX, y: event.offsetY };
  }
  private mouseDown(event: MouseEvent) {
    this.mousePosition = { x: event.offsetX, y: event.offsetY };
    this.hasPendingClick = true;
  }
  private mouseUp(event: MouseEvent) {
    this.hasPendingClick = false;
    this.activeIndex = null;
  }
}

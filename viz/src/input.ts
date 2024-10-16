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
  private isClicking: boolean = false;

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
    const distances = scaledLocations.map((p) =>
      Math.sqrt(Math.pow(mx - p.x, 2) + Math.pow(my - p.y, 2))
    );
    const minIndex = indexOfSmallest(distances);

    const minDistanceToInteract = Number.POSITIVE_INFINITY; //  Pixels
    if (distances[minIndex] < minDistanceToInteract) {
      // Convert to an incident angle.
      const angle = Math.atan2(my - scaledLocations[minIndex].y, mx - scaledLocations[minIndex].x);
      return new MouseInteraction(minIndex, angle, this.isClicking);
    }
    return null;
  }

  private mouseEnter(event: MouseEvent) {}
  private mouseLeave(event: MouseEvent) {
    this.mousePosition = null;
    this.isClicking = false;
  }
  private mouseMove(event: MouseEvent) {
    this.mousePosition = { x: event.offsetX, y: event.offsetY };
  }
  private mouseDown(event: MouseEvent) {
    this.isClicking = true;
  }
  private mouseUp(event: MouseEvent) {
    this.isClicking = false;
  }
}

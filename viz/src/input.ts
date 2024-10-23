// Copyright 2024 Gareth Cross.
import {
  massLocationsFromState,
  Point,
  ScaleAndTranslate,
  SingleCartPoleState,
  SingleCartPoleParams,
  indexOfSmallest
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

// Handle user interaction with the cart-pole system via mouse events.
export class MouseHandler {
  private canvas: HTMLCanvasElement;
  private mousePosition: Point | null = null;
  private isClicked: boolean = false;
  private activeIndex: number | null = null;

  constructor() {
    const canvas: HTMLCanvasElement = document.getElementById(
      'cartPoleCanvas'
    ) as HTMLCanvasElement;
    this.canvas = canvas;

    canvas.addEventListener('mouseenter', (e) => this.mouseEnter(e));
    canvas.addEventListener('mouseleave', (e) => this.mouseLeave(e));

    if (!window.matchMedia('(pointer: coarse)').matches) {
      canvas.addEventListener('mousemove', (e) => this.mouseMove(e));
      canvas.addEventListener('mousedown', (e) => this.mouseDown(e));
      canvas.addEventListener('mouseup', (e) => this.mouseUp());
    } else {
      canvas.addEventListener('touchstart', (e) => this.touchStart(e));
      canvas.addEventListener('touchend', (e) => this.touchEnd());
      canvas.addEventListener('touchmove', (e) => this.touchMove(e));
    }
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
      if (this.isClicked) {
        this.activeIndex = minIndex;
      }
      return minIndex;
    }
    return this.activeIndex;
  }

  private mouseEnter(event: MouseEvent) {}
  private mouseLeave(event: MouseEvent) {
    this.mousePosition = null;
    this.isClicked = false;
    this.activeIndex = null;
  }
  private mouseMove(event: MouseEvent) {
    this.mousePosition = { x: event.offsetX, y: event.offsetY };
  }
  private mouseDown(event: MouseEvent) {
    this.mousePosition = { x: event.offsetX, y: event.offsetY };
    this.isClicked = true;
  }
  private mouseUp() {
    this.isClicked = false;
    this.activeIndex = null;
  }

  private touchStart(event: TouchEvent) {
    const domRect = this.canvas.getBoundingClientRect();
    const touch = event.touches[0];
    this.mousePosition = {
      x: touch.clientX - domRect.left,
      y: touch.clientY - domRect.top
    };
    this.isClicked = true;
  }
  private touchMove(event: TouchEvent) {
    const domRect = this.canvas.getBoundingClientRect();
    const touch = event.touches[0];
    this.mousePosition = {
      x: touch.clientX - domRect.left,
      y: touch.clientY - domRect.top
    };
  }
  private touchEnd() {
    this.mouseUp();
    // Set this null so the arrow hides.
    this.mousePosition = null;
  }
}

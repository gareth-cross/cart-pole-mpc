// Fixed parameters of the pendulum system.
export class PendulumParams {
  public m_b: number;
  public m_1: number;
  public m_2: number;
  public l_1: number;
  public l_2: number;
  public g: number;

  constructor(
    m_b: number,
    m_1: number,
    m_2: number,
    l_1: number,
    l_2: number,
    g: number
  ) {
    this.assertPositive(m_b, 'm_b');
    this.assertPositive(m_1, 'm_1');
    this.assertPositive(m_2, 'm_2');
    this.assertPositive(l_1, 'l_1');
    this.assertPositive(l_2, 'l_2');
    this.assertPositive(g, 'g');

    this.m_b = m_b;
    this.m_1 = m_1;
    this.m_2 = m_2;
    this.l_1 = l_1;
    this.l_2 = l_2;
    this.g = g;
  }

  private assertPositive(value: number, paramName: string): void {
    if (value <= 0) {
      throw new Error(`${paramName} must be a positive number`);
    }
  }
}

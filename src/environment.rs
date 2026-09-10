use rapier3d::prelude::Vector;
pub struct Environment<'a> {
    objects: Vec<&'a Vector>,
}

impl<'a> Default for Environment<'a> {
    fn default() -> Self {
        Self {
            objects: Vec::new(),
        }
    }
}

impl<'a> Environment<'a> {
    fn push(&mut self, input_vector: &'a Vector) {
        self.objects.push(input_vector);
    }
}
